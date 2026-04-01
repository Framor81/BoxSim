# BoxSim

Unreal Engine 5 + UnrealCV: pose, drive, map building.

```bash
pip install -r requirements.txt
```

**Features**
- **Pose** — Poll pawn location (X, Y) and yaw at 5 Hz from Unreal.
- **Drive** — Run a sequence of W/A/S/D key presses (declare in `drive.py` PATH); keys sent to Unreal via UnrealCV.
- **Map from screenshot** — Capture top-down ortho from Unreal, draw obstacles (polygon/brush/box), save map.png + map.json.
- **Map from scratch** — Same drawing tools on a blank grid; optional live pose overlay.
- **Object list / debug** — `--list-objects` to see pawn object name; `--debug` to see raw UnrealCV commands and responses.

| Command | What |
|---------|------|
| `python main.py` | Pose at 5 Hz. Ctrl+C stop. |
| `python main.py --list-objects` | List object names (use as `UNREALCV_PAWN`) |
| `python main.py --debug` | Raw UnrealCV responses |
| `python drive.py` | Run PATH in drive.py (W/A/S/D sent to Unreal) |
| `python build.py screenshot` | Top-down from Unreal, annotate, save |
| `python build.py manual` | Blank canvas, draw obstacles |
| `python build.py manual --unreal` | Same + connect to Unreal (robot overlay) |


Env: `UNREALCV_PAWN` if your pawn object name differs (use object name, not display name).

For screenshot capture with FusionCamSensor (custom UnrealCV), lit uses a **non-zero** camera id — set `BOXSIM_UNREALCV_CAMERA_ID=1` if needed, or rely on `vget /cameras` (camera `0` is the pawn, not the lit sensor).

Robot overlay: map coords use **native lit** pixels (`scale = capture_width / ortho_width`); `_map_pixel_to_screen` multiplies by `window / capture_native` so polys, grid, and robot stay aligned when the screenshot is stretched. With `pose_swap_xy`, heading uses `π/2 − yaw` so UE forward matches the swapped axes (tune with `pose_yaw_offset_deg` if still off). `capture_native_*`, `pose_*`, and optional `origin` in JSON as before.

Map controls: Tools and terrain on the left sidebar (1/2/3=Poly/Brush/Box, A/S/D=obstacle/drivable/cut, W=goal E=path). Click first path point again or right-click to close. Ctrl+S=Save, Ctrl+Z=Undo. Grid: center 0,0, range -500 to 500; cut restores to grid (or background image). Save stores geometry in map.json (obstacles, drivable, erase_polygons) for rebuild.
