# Map builder: screenshot | manual | frompng. Use --unreal to connect for manual (robot overlay).

import argparse
import sys

from agent import UnrealAgent
from builder import ManualMapBuilder, OutlinePngMapBuilder, ScreenshotMapBuilder


def main() -> int:
    parser = argparse.ArgumentParser(description="BoxSim map builder")
    sub = parser.add_subparsers(dest="cmd", required=True)

    p_ss = sub.add_parser("screenshot", help="Capture ortho from Unreal and trace on lit image")
    p_ss.add_argument(
        "-ni",
        "--no-icon",
        "--no-agent-icon",
        dest="no_agent_icon",
        action="store_true",
        help="Do not draw the agent icon on the screenshot",
    )

    p_man = sub.add_parser("manual", help="Blank grid (world-sized from config)")
    p_man.add_argument("--unreal", action="store_true", help="Connect to Unreal for robot overlay")

    p_png = sub.add_parser(
        "frompng",
        help="Trace on an outline image under outlines/; JSON uses plan inches (no *_w.json)",
    )
    p_png.add_argument(
        "image",
        nargs="?",
        help="Image under outlines/ (e.g. 1.png); optional if --image is set",
    )
    p_png.add_argument(
        "-i",
        "--image",
        dest="image_opt",
        metavar="NAME",
        help="Outline image name under outlines/ (alternative to positional)",
    )
    p_png.add_argument(
        "--width-in",
        type=float,
        required=True,
        metavar="IN",
        help="Horizontal extent of the plan in inches (maps to image width)",
    )
    p_png.add_argument(
        "--length-in",
        type=float,
        required=True,
        metavar="IN",
        help="Vertical extent of the plan in inches (maps to image height; y=0 at bottom)",
    )
    p_png.add_argument(
        "-o",
        "--out",
        default="data/maps/outline_trace",
        metavar="PREFIX",
        help="Output path prefix without extension (default: data/maps/outline_trace)",
    )

    args = parser.parse_args()
    agent = UnrealAgent()

    if args.cmd == "screenshot":
        if not agent.connect():
            print("Failed to connect to UnrealCV (port 9000)", file=sys.stderr)
            return 1
        ScreenshotMapBuilder(save_path_prefix="data/maps/map").run(
            agent, show_agent_icon=not bool(getattr(args, "no_agent_icon", False))
        )
    elif args.cmd == "manual":
        if args.unreal:
            agent.connect()
        ManualMapBuilder(save_path_prefix="data/maps/manual_map").run(
            agent if agent.is_connected() else None
        )
    elif args.cmd == "frompng":
        name = args.image_opt or args.image
        if not name:
            print(
                "frompng: provide an image name (positional or --image), e.g.\n"
                "  python build.py frompng 1.png --width-in 144 --length-in 96",
                file=sys.stderr,
            )
            return 1
        try:
            OutlinePngMapBuilder(
                outline_name=name,
                width_inches=args.width_in,
                length_inches=args.length_in,
                save_path_prefix=args.out,
            ).run()
        except (FileNotFoundError, IOError, ValueError) as e:
            print(str(e), file=sys.stderr)
            return 1
    else:
        return 1

    if agent.is_connected():
        agent.disconnect()
    return 0


if __name__ == "__main__":
    sys.exit(main())
