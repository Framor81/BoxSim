"""
Run map builder: screenshot from Unreal or manual canvas.

  python build.py screenshot   # capture top-down, then annotate
  python build.py manual      # blank canvas, draw boxes yourself
  UNREALCV_HOST=192.168.1.50 python build.py screenshot
"""

import os
import sys

from agent import UnrealAgent
from builder import ManualMapBuilder, ScreenshotMapBuilder


def main() -> int:
    if len(sys.argv) < 2 or sys.argv[1] not in ("screenshot", "manual"):
        print("Usage: python build.py screenshot | manual")
        return 1
    mode = sys.argv[1]
    host = os.environ.get("UNREALCV_HOST", "localhost")
    port = int(os.environ.get("UNREALCV_PORT", "9000"))
    agent = UnrealAgent(host=host, port=port)
    if mode == "screenshot":
        if not agent.connect():
            print("Failed to connect to UnrealCV (port 9000)", file=sys.stderr)
            return 1
        ScreenshotMapBuilder(save_path_prefix="data/maps/map").run(agent)
    else:
        agent.connect()
        ManualMapBuilder(save_path_prefix="data/maps/manual_map").run(
            agent if agent.is_connected() else None
        )
    if agent.is_connected():
        agent.disconnect()
    return 0


if __name__ == "__main__":
    sys.exit(main())
