# Map builder: screenshot | manual. Use --unreal to connect for manual (robot overlay).

import sys

from agent import UnrealAgent
from builder import ManualMapBuilder, ScreenshotMapBuilder


def main() -> int:
    if len(sys.argv) < 2 or sys.argv[1] not in ("screenshot", "manual"):
        print("Usage: python build.py screenshot [-ni|--no-icon|--no-agent-icon] | manual [--unreal]")
        return 1
    mode = sys.argv[1]
    use_unreal = "--unreal" in sys.argv
    no_agent_icon = any(f in sys.argv for f in ("-ni", "--no-icon", "--no-agent-icon"))
    agent = UnrealAgent()
    if mode == "screenshot":
        if not agent.connect():
            print("Failed to connect to UnrealCV (port 9000)", file=sys.stderr)
            return 1
        ScreenshotMapBuilder(save_path_prefix="data/maps/map").run(
            agent, show_agent_icon=not no_agent_icon
        )
    else:
        if use_unreal:
            agent.connect()
        ManualMapBuilder(save_path_prefix="data/maps/manual_map").run(
            agent if agent.is_connected() else None
        )
    if agent.is_connected():
        agent.disconnect()
    return 0


if __name__ == "__main__":
    sys.exit(main())
