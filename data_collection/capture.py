# Lit-image capture helpers (UnrealCV `vget /camera/<id>/lit <path>`).
#
# IMPORTANT: in UnrealCV camera 0 is the *possessed player view* and is
# usually NOT writable via `vset /camera/0/location` (engine restores it each
# tick — see UnrealCV issues #198 and #232). For collection we therefore
# prefer the smallest *non-zero*, *responsive* camera id (typically a
# FusionCameraActor at id 1). Override via `--camera-id` or
# BOXSIM_UNREALCV_CAMERA_ID.

from __future__ import annotations

import os
import re
from pathlib import Path

from agent import UnrealAgent


def _parse_xyz(raw: str | None) -> tuple[float, float, float] | None:
    if not raw:
        return None
    nums = re.findall(r"-?\d+(?:\.\d+)?", str(raw))
    if len(nums) < 3:
        return None
    try:
        return (float(nums[0]), float(nums[1]), float(nums[2]))
    except ValueError:
        return None


def resolve_lit_camera_id(
    agent: UnrealAgent, *, debug: bool = False
) -> int | None:
    env = os.environ.get("BOXSIM_UNREALCV_CAMERA_ID")
    if env is not None and env.strip() != "":
        try:
            return int(env)
        except ValueError:
            pass

    if not agent.is_connected():
        return None

    try:
        raw = agent.client.request("vget /cameras")
    except Exception:
        raw = None
    ids: list[int] = []
    if raw:
        for tok in raw.replace(",", " ").split():
            try:
                ids.append(int(tok.strip()))
            except ValueError:
                continue
    if not ids:
        ids = [0, 1, 2, 3]
    ids = sorted(set(ids))

    # Filter out phantom ids that don't actually respond (vget returns
    # 'error Invalid sensor id' for them in some builds).
    responsive: list[int] = []
    for cid in ids:
        try:
            loc_raw = agent.client.request(f"vget /camera/{cid}/location")
        except Exception:
            continue
        if not loc_raw or str(loc_raw).lower().startswith("error"):
            continue
        if _parse_xyz(loc_raw) is None:
            continue
        responsive.append(cid)
    if not responsive:
        responsive = ids  # fall back to whatever vget /cameras gave us

    # Prefer the smallest *non-zero* responsive id (camera 0 is the locked
    # player view in standard UnrealCV builds and ignores vset).
    non_zero = [c for c in responsive if c != 0]
    chosen = non_zero[0] if non_zero else responsive[0]
    if debug:
        print(f"  camera ids responsive: {responsive} (preferred non-zero: {non_zero}) -> chose {chosen}")
    return chosen


def capture_lit_image(
    agent: UnrealAgent,
    out_path: str | Path,
    *,
    camera_id: int | None = None,
    debug: bool = False,
) -> bool:
    """Save a lit PNG from UnrealCV to `out_path`. Returns True on success."""
    if not agent.is_connected():
        return False
    cam = camera_id
    if cam is None:
        cam = resolve_lit_camera_id(agent, debug=debug)
    if cam is None:
        return False
    out = Path(out_path).resolve()
    out.parent.mkdir(parents=True, exist_ok=True)
    cmd = f"vget /camera/{cam}/lit {out.as_posix()}"
    if debug:
        print(f"  >> {cmd}")
    try:
        resp = agent.client.request(cmd)
    except Exception as e:
        if debug:
            print(f"  !! {e}")
        return False
    if debug:
        print(f"  << {resp!r}")
    # UnrealCV often echoes the requested path even when write fails; verify on disk.
    ok = out.exists()
    if debug and not ok:
        print(f"  !! capture file missing after vget: {out}")
    return ok
