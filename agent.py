"""
UnrealCV connection and pawn pose.
Pawn is identified by name: set UNREALCV_PAWN to match your Blueprint or instance name.
"""

from __future__ import annotations

import os
import time
from typing import NamedTuple

from unrealcv import Client

DEFAULT_HOST = "localhost"
DEFAULT_PORT = 9000


def _pawn_name() -> str:
    return os.environ.get("UNREALCV_PAWN", "BP_MyPlayer_Pawn_C_1")


class PawnPose(NamedTuple):
    """Pose: X, Y (Unreal), Yaw (degrees)."""
    x: float
    y: float
    yaw: float


class UnrealAgent:
    """UnrealCV client; pose and optional camera/screenshot commands."""

    def __init__(self, host: str = DEFAULT_HOST, port: int = DEFAULT_PORT) -> None:
        self._host = host
        self._port = port
        self._client = Client((host, port), "inet")
        self._connected = False

    def connect(self, timeout: float = 5.0) -> bool:
        if self._client.connect(timeout=int(timeout)):
            self._connected = True
            return True
        self._connected = False
        return False

    def disconnect(self) -> None:
        self._client.disconnect()
        self._connected = False

    def is_connected(self) -> bool:
        return self._connected and self._client.isconnected()

    @property
    def client(self):
        return self._client

    def check_status(self) -> str | None:
        if not self.is_connected():
            return None
        try:
            return self._client.request("vget /unrealcv/status")
        except Exception:
            return None

    def list_objects(self) -> list[str]:
        """Return object names visible to UnrealCV. Use the OBJECT NAME (e.g. BP_MyPlayer_Pawn_C_1), not the display name."""
        if not self.is_connected():
            return []
        try:
            r = self._client.request("vget /objects")
            if not r:
                return []
            names = []
            for sep in ("\n", ",", " "):
                if sep in r:
                    names = [s.strip() for s in r.split(sep) if s.strip()]
                    break
            if not names:
                names = [r.strip()] if r.strip() else []
            return names
        except Exception:
            return []

    def get_pawn_pose(self, *, debug: bool = False) -> PawnPose | None:
        if not self.is_connected():
            return None
        name = _pawn_name()
        loc, rot = None, None

        def _log(s: str) -> None:
            if debug:
                print(f"  {s}")

        _log(f"Pawn name: {name!r}")

        # Try vbp (Blueprint) first
        _log("Trying vbp (Blueprint)...")
        try:
            cmd_loc = f"vbp {name} GetActorLocation"
            cmd_rot = f"vbp {name} GetActorRotation"
            _log(f"  Send: {cmd_loc}")
            loc = self._client.request(cmd_loc)
            _log(f"  Recv: {loc!r}")
            _log(f"  Send: {cmd_rot}")
            rot = self._client.request(cmd_rot)
            _log(f"  Recv: {rot!r}")
        except Exception as e:
            _log(f"  vbp exception: {e}")
        x, y, z = self._parse_location(loc)
        pitch, yaw, roll = self._parse_rotation(rot)
        _log(f"  Parsed location: x={x} y={y} z={z}")
        _log(f"  Parsed rotation: pitch={pitch} yaw={yaw} roll={roll}")
        if x is not None and yaw is not None:
            return PawnPose(x=x, y=y, yaw=yaw)

        # Fallback: vget /object/name/location and rotation
        _log("vbp failed or bad parse. Trying vget /object/name/...")
        try:
            cmd_loc = f"vget /object/{name}/location"
            cmd_rot = f"vget /object/{name}/rotation"
            _log(f"  Send: {cmd_loc}")
            loc = self._client.request(cmd_loc)
            _log(f"  Recv: {loc!r}")
            _log(f"  Send: {cmd_rot}")
            rot = self._client.request(cmd_rot)
            _log(f"  Recv: {rot!r}")
        except Exception as e:
            _log(f"  vget exception: {e}")
        x, y, z = self._parse_location(loc)
        pitch, yaw, roll = self._parse_rotation(rot)
        _log(f"  Parsed location: x={x} y={y} z={z}")
        _log(f"  Parsed rotation: pitch={pitch} yaw={yaw} roll={roll}")
        if x is None or yaw is None:
            _log("Still no valid pose. Use OBJECT NAME (e.g. BP_MyPlayer_Pawn_C_1) for UNREALCV_PAWN, not display name.")
        else:
            return PawnPose(x=x, y=y, yaw=yaw)
        return None

    @staticmethod
    def _parse_location(r: str | None) -> tuple[float | None, float | None, float | None]:
        if not r or not r.strip():
            return None, None, None
        parts = r.strip().split()
        if len(parts) < 3:
            return None, None, None
        try:
            return float(parts[0]), float(parts[1]), float(parts[2])
        except ValueError:
            return None, None, None

    @staticmethod
    def _parse_rotation(r: str | None) -> tuple[float | None, float | None, float | None]:
        if not r or not r.strip():
            return None, None, None
        parts = r.strip().split()
        if len(parts) < 3:
            return None, None, None
        try:
            return float(parts[0]), float(parts[1]), float(parts[2])
        except ValueError:
            return None, None, None


def run_pose_loop(agent: UnrealAgent, rate_hz: float = 5.0, *, debug: bool = False, hint_on_fail: bool = True) -> None:
    """Poll pose at rate_hz, print until Ctrl+C."""
    interval = 1.0 / rate_hz
    hint_printed = False
    while True:
        t0 = time.perf_counter()
        pose = agent.get_pawn_pose(debug=debug)
        if pose is not None:
            print(f"pose x={pose.x:.2f} y={pose.y:.2f} yaw={pose.yaw:.2f}")
        else:
            print("pose (unavailable)")
            if hint_on_fail and not hint_printed:
                print("  Tip: Use OBJECT NAME (e.g. BP_MyPlayer_Pawn_C_1) for UNREALCV_PAWN, not display name. --list-objects to see names, --debug for raw responses.")
                hint_printed = True
        elapsed = time.perf_counter() - t0
        if interval - elapsed > 0:
            time.sleep(interval - elapsed)
