# Teleport / read pawn pose over UnrealCV.
#
# Uses the built-in object endpoints first (vset /object/<name>/location|rotation),
# then falls back to Blueprint calls. The pawn name comes from UNREALCV_PAWN
# (same env var main.py uses) or the explicit argument.

from __future__ import annotations

import os
from typing import Tuple

from agent import UnrealAgent


def _pawn_name(explicit: str | None) -> str:
    if explicit:
        return explicit
    return os.environ.get("UNREALCV_PAWN", "BP_MyPlayer_Pawn_C_1")


def _make_send(agent: UnrealAgent, debug: bool):
    def _send(cmd: str) -> str | None:
        if debug:
            print(f"  >> {cmd}")
        try:
            r = agent.client.request(cmd)
            if debug:
                print(f"  << {r!r}")
            return r
        except Exception as e:
            if debug:
                print(f"  !! {e}")
            return None

    return _send


def set_pawn_rotation(
    agent: UnrealAgent,
    yaw_deg: float,
    *,
    pawn: str | None = None,
    debug: bool = False,
) -> bool:
    """Set pawn yaw only (no location change)."""
    if not agent.is_connected():
        return False
    name = _pawn_name(pawn)
    _send = _make_send(agent, debug)
    cmd = f"vset /object/{name}/rotation 0.0 {yaw_deg:.4f} 0.0"
    if _ok(_send(cmd)):
        return True
    return _ok(_send(f"vbp {name} SetActorRotation 0.0 {yaw_deg:.4f} 0.0"))


def set_pawn_location(
    agent: UnrealAgent,
    x: float,
    y: float,
    *,
    z: float = 0.0,
    pawn: str | None = None,
    debug: bool = False,
) -> bool:
    """Set pawn world location only (no rotation change)."""
    if not agent.is_connected():
        return False
    name = _pawn_name(pawn)
    _send = _make_send(agent, debug)
    cmd = f"vset /object/{name}/location {x:.4f} {y:.4f} {z:.4f}"
    if _ok(_send(cmd)):
        return True
    return _ok(_send(f"vbp {name} SetActorLocation {x:.4f} {y:.4f} {z:.4f}"))


def teleport_pawn(
    agent: UnrealAgent,
    x: float,
    y: float,
    yaw_deg: float,
    *,
    z: float = 0.0,
    pawn: str | None = None,
    debug: bool = False,
) -> bool:
    """Set pawn world location and yaw atomically (rotation first, then location).

    Prefer `set_pawn_rotation` / `set_pawn_location` when you want to capture
    rotation and translation as separate frames.
    """
    ok_rot = set_pawn_rotation(agent, yaw_deg, pawn=pawn, debug=debug)
    ok_loc = set_pawn_location(agent, x, y, z=z, pawn=pawn, debug=debug)
    return ok_rot and ok_loc


def get_pawn_pose_loc(
    agent: UnrealAgent, *, pawn: str | None = None
) -> Tuple[float, float, float, float] | None:
    """Returns (x, y, z, yaw_deg) from a fresh read, or None."""
    pose = agent.get_pawn_pose()
    if pose is None:
        return None
    return (pose.x, pose.y, pose.z, pose.yaw)


def _ok(resp: str | None) -> bool:
    if resp is None:
        return False
    s = resp.strip().lower()
    if s in ("", "ok"):
        return True
    # UnrealCV returns "error ..." on failure; anything else (e.g. echoed
    # values) we consider non-fatal.
    return not s.startswith("error")
