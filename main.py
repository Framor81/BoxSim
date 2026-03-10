"""
Pose polling: connect to UnrealCV and print pawn pose at 5 Hz until Ctrl+C.

  python main.py                  # normal
  python main.py --list-objects   # print object names (find your pawn)
  python main.py --debug          # show raw UnrealCV response when pose fails
  UNREALCV_PAWN=MyPawn_0 python main.py   # use this pawn name
"""

import os
import sys

from agent import UnrealAgent, run_pose_loop

POSE_RATE_HZ = 5.0


def main() -> int:
    if "--list-objects" in sys.argv:
        host = os.environ.get("UNREALCV_HOST", "localhost")
        port = int(os.environ.get("UNREALCV_PORT", "9000"))
        agent = UnrealAgent(host=host, port=port)
        if not agent.connect():
            print("Failed to connect.", file=sys.stderr)
            return 1
        names = agent.list_objects()
        print("Objects visible to UnrealCV:")
        for n in names:
            print(f"  {n}")
        print("")
        print("Use the OBJECT NAME (e.g. BP_MyPlayer_Pawn_C_1) for UNREALCV_PAWN, not the display name.")
        print("Example: UNREALCV_PAWN=BP_MyPlayer_Pawn_C_1 python main.py")
        agent.disconnect()
        return 0

    debug = "--debug" in sys.argv
    host = os.environ.get("UNREALCV_HOST", "localhost")
    port = int(os.environ.get("UNREALCV_PORT", "9000"))
    pawn = os.environ.get("UNREALCV_PAWN", "BP_MyPlayer_Pawn_C_1")
    agent = UnrealAgent(host=host, port=port)
    print(f"Connecting to UnrealCV at {host}:{port} ...")
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
