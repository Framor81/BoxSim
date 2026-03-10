"""
Run map builder: screenshot from Unreal or manual canvas.

  python build.py screenshot   # capture top-down, then annotate
  python build.py manual      # blank canvas, draw boxes yourself
"""

import sys

from agent import UnrealAgent
from builder import ScreenshotMapBuilder, ManualMapBuilder


def main() -> int:
    if len(sys.argv) < 2 or sys.argv[1] not in ("screenshot", "manual"):
        print("Usage: python build.py screenshot | manual")
        return 1
    mode = sys.argv[1]
    agent = UnrealAgent(host="localhost", port=9000)
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
