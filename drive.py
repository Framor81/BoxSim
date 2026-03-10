# Run PATH (key, seconds). Keys sent to Unreal via UnrealCV.

import sys

from agent import UnrealAgent


PATH = [("W", 5), ("A", 5), ("W", 5)]  # (key, seconds). W/A/S/D.

KEY_METHODS = {
    "W": UnrealAgent.move_forward,
    "S": UnrealAgent.move_backward,
    "A": UnrealAgent.turn_left,
    "D": UnrealAgent.turn_right,
}


def main() -> int:
    agent = UnrealAgent()
    print("Connecting to UnrealCV (localhost:9000) ...")
    if not agent.connect():
        print("Failed to connect.", file=sys.stderr)
        return 1
    print("Running path...")
    try:
        for key, duration in PATH:
            method = KEY_METHODS.get(key.upper())
            if method is None:
                print(f"  Unknown key {key!r}, skipping")
                continue
            print(f"  {key} for {duration}s")
            method(agent, duration)
        print("Done.")
    finally:
        agent.disconnect()
    return 0


if __name__ == "__main__":
    sys.exit(main())
