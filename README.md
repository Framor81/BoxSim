# BoxSim

Unreal Engine 5 + UnrealCV: pose, drive, map building.

```bash
pip install -r requirements.txt
```

**Capture config (framing and pose)**  
When `config/boxsim.json` is missing, [config/boxsim.example.json](config/boxsim.example.json) is used. It frames a fixed UE **AABB** from corners **(-720, -350)**, **(-720, 750)**, **(210, 750)**, **(210, -350)** — i.e. **[xmin, xmax, ymin, ymax] = [-720, 210, -350, 750]**, world size **930 × 1100**, center **(-255, 200)**. Copy to `config/boxsim.json` (gitignored) or set `BOXSIM_CONFIG` to change it. Use **`capture.mode": "pawn_xy"`** if you want the view centered on the pawn with ortho from `ortho_width` / `ortho_height` instead.

- **`capture.mode`**: `pawn_xy` centers the ortho camera on the pawn (same XY) and uses `ortho_width` / `ortho_height` for how much world fits in the shot. `aabb` uses `capture.bounds` as `[xmin, xmax, ymin, ymax]` in UE world XY: camera looks at the box center and ortho spans match that rectangle.
- **`capture.camera_z_offset`**: height of the virtual lit camera above the pawn Z. For an **orthographic** top-down shot this does **not** change zoom; it only shifts along the view axis. **Zoom / field of view** is **`ortho_width` and `ortho_height`** (and UE/sensor aspect).
- **`pose`**: Written into map metadata for the robot overlay:
  - **`pose_swap_xy`**: when **true**, pawn **UE Y** drives **horizontal** pygame motion (+UE **Y** → +screen **X**), and UE **X** drives vertical — matches many top-down UE5 shots.
  - **`pose_yaw_offset_deg`**: adds to Unreal yaw before drawing the triangle (try **90** or **-90** if forward points wrong).
  - **`map_mirror_x` / `map_mirror_y`**: flip map ↔ screen along one axis if a world direction still moves backward in pygame.
  - **`pose_pixel_flip_y`**: vertical flip in map-pixel space (screenshot vs manual).

Env overrides (when set) win over the file: `BOXSIM_CAPTURE_WORLD_BOUNDS`, `BOXSIM_CAPTURE_USE_PAWN_XY`, `BOXSIM_ORTHO_WIDTH`, `BOXSIM_ORTHO_HEIGHT`, `BOXSIM_CAMERA_Z_OFFSET`, `BOXSIM_POSE_*`, etc. See [builder/capture_config.py](builder/capture_config.py).

**Agent / grid vs background drift** — The robot must use the **same** UE `(x, y)` as the grid labels. Keep **`pose_swap_xy": false`** unless your top-down image maps UE **X** to vertical and **Y** to horizontal on the bitmap. (Older builds defaulted swap on for screenshots and could place the pawn at `(y, x)` while the grid stayed in `(x, y)`.) If the **photo** still does not line up with the grid at a known landmark, try in order: **`capture.swap_ortho_width_height`** (`BOXSIM_SWAP_ORTHO_WIDTH_HEIGHT=1`) if UE pairs ortho extents to the wrong image axis; **`capture.transpose_lit_image`** (`BOXSIM_TRANSPOSE_LIT_IMAGE=1`) if rows/columns vs world X/Y are swapped in the PNG; **`capture.origin_world_adjust": [dx, dy]`** (`BOXSIM_ORIGIN_WORLD_ADJUST=dx,dy`) to nudge the registered world center in small world units. **Recapture** after changing these so `map.json` matches the processed image.

Fixed world rectangle (four UE corners as min/max), in `config/boxsim.json`:

```json
{
  "capture": {
    "mode": "aabb",
    "bounds": [-720, 210, -350, 750],
    "ortho_width": 2000,
    "ortho_height": 2000
  },
  "pose": {
    "pose_swap_xy": true,
    "pose_pixel_flip_y": false,
    "pose_yaw_offset_deg": 0
  }
}
```

With `aabb`, `ortho_width`/`ortho_height` in the file are ignored for the Unreal `vset` (spans come from `bounds`); they still inform defaults if you switch back to `pawn_xy`.

**World coordinates on the map**  
After a screenshot capture, `map.json` stores `origin` (world XY at the **center** of the image), `scale_x` / `scale_y` (native pixels per world unit along image X and Y), and optional `capture_world_bounds` / `world_bounds`. Native-lit offsets use `world_to_pixel` in [map_utils.py](map_utils.py); the viewer scales from native resolution to the pygame window. Older maps with only `scale` still load (`scale_y` defaults to `scale`).

If a **top-down shot** shows landmarks hundreds of world units away from grid labels (e.g. UE `(-50,-90)` appears at tick `(-550,560)`), check **`origin`**: it must be the capture center **(~(-255, 200)** for the default AABB), not **`[0,0]`** from an old manual map. The viewer now **re-syncs** `capture_native_*` and `scale_*` from the actual PNG shape, and resets **`origin`** from `capture_bounds_center` / bounds when `origin` is zero or **> 25% of the span** away from that center. **Save** (Ctrl+S) to persist the fixed `origin` in `map.json`.

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

Robot overlay: map coords use **native lit** pixels (`scale_x` / `scale_y` from ortho spans and bitmap size); `_map_pixel_to_screen` multiplies by `window / capture_native` so polys, grid, and robot stay aligned when the screenshot is stretched. With `pose_swap_xy`, heading uses `π/2 − yaw` so UE forward matches the swapped axes (tune with `pose_yaw_offset_deg` if still off).

Map controls: Tools and terrain on the left sidebar (1/2/3=Poly/Brush/Box, A/S/D=obstacle/drivable/cut, W=goal E=path). Click first path point again or right-click to close. Ctrl+S=Save, Ctrl+Z=Undo. Grid extent comes from `world_bounds` / `capture_world_bounds` or from `ortho_*` and `origin` in JSON; cut restores to grid (or background image). Save stores geometry in map.json (obstacles, drivable, erase_polygons) for rebuild.
