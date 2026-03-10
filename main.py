"""
Pose polling: connect to UnrealCV and print pawn pose at 5 Hz until Ctrl+C.
"""

import sys

from agent import UnrealAgent, run_pose_loop

POSE_RATE_HZ = 5.0


def main() -> int:
    agent = UnrealAgent(host="localhost", port=9000)
    print("Connecting to UnrealCV at localhost:9000 ...")
    if not agent.connect():
        print("Failed to connect. Is Unreal running with UnrealCV on port 9000?", file=sys.stderr)
        return 1
    status = agent.check_status()
    print(f"Connected. Status: {status}")
    print(f"Polling BP_MyPlayer_Pawn_C pose at {POSE_RATE_HZ} Hz (Ctrl+C to stop).")
    try:
        run_pose_loop(agent, rate_hz=POSE_RATE_HZ)
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        agent.disconnect()
    return 0


if __name__ == "__main__":
    sys.exit(main())
