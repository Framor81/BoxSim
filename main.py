# Pose polling. --list-objects / --debug. UNREALCV_PAWN for pawn name.

import os
import sys

from agent import UnrealAgent, run_pose_loop

POSE_RATE_HZ = 5.0


def main() -> int:
    if "--list-objects" in sys.argv:
        agent = UnrealAgent()
        if not agent.connect():
            print("Failed to connect.", file=sys.stderr)
            return 1
        names = agent.list_objects()
        print("Objects visible to UnrealCV:")
        for n in names:
            print(f"  {n}")
        print("")
        print("Use object name for UNREALCV_PAWN.")
        agent.disconnect()
        return 0

    debug = "--debug" in sys.argv
    pawn = os.environ.get("UNREALCV_PAWN", "BP_MyPlayer_Pawn_C_1")
    agent = UnrealAgent()
    print("Connecting to UnrealCV (localhost:9000) ...")
    if not agent.connect():
        print("Failed to connect. Is Unreal running with UnrealCV on port 9000?", file=sys.stderr)
        return 1
    status = agent.check_status()
    print(f"Connected. Status: {status}")
    print(f"Pawn name: {pawn} (set UNREALCV_PAWN to override)")
    print(f"Polling pose at {POSE_RATE_HZ} Hz (Ctrl+C to stop).")
    try:
        run_pose_loop(agent, rate_hz=POSE_RATE_HZ, debug=debug, hint_on_fail=not debug)
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        agent.disconnect()
    return 0


if __name__ == "__main__":
    sys.exit(main())
