"""
UnrealCV connection and pawn pose (BP_MyPlayer_Pawn_C).
"""

from __future__ import annotations

import time
from typing import NamedTuple

from unrealcv import Client

DEFAULT_HOST = "localhost"
DEFAULT_PORT = 9000
PAWN_BLUEPRINT = "BP_MyPlayer_Pawn_C"


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

    def get_pawn_pose(self) -> PawnPose | None:
        if not self.is_connected():
            return None
        try:
            loc = self._client.request(f"vbp {PAWN_BLUEPRINT} GetActorLocation")
            rot = self._client.request(f"vbp {PAWN_BLUEPRINT} GetActorRotation")
        except Exception:
            return None
        x, y, z = self._parse_location(loc)
        pitch, yaw, roll = self._parse_rotation(rot)
        if x is None or yaw is None:
            return None
        return PawnPose(x=x, y=y, yaw=yaw)

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


def run_pose_loop(agent: UnrealAgent, rate_hz: float = 5.0) -> None:
    """Poll pose at rate_hz, print until Ctrl+C."""
    interval = 1.0 / rate_hz
    while True:
        t0 = time.perf_counter()
        pose = agent.get_pawn_pose()
        if pose is not None:
            print(f"pose x={pose.x:.2f} y={pose.y:.2f} yaw={pose.yaw:.2f}")
        else:
            print("pose (unavailable)")
        elapsed = time.perf_counter() - t0
        if interval - elapsed > 0:
            time.sleep(interval - elapsed)
