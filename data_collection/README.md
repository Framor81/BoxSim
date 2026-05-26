# `data_collection/` — Spline-following data collection

Drive the pawn along the path exported from the
[UE5 Path Tape prototype](../../UE5PathTapePrototype) and record one frame
per step: pose + optional lit PNG + label.

## What it does

1. Loads the `"path"` polyline from a BoxSim map JSON (e.g. `data/maps/map_w.json`).
   Points are **UE world XY (cm)** with `"path_space": "world"`.
2. Builds a **walkable curve** and measures **arc length** along it:
   - **`catmull_rom` (default):** a smooth Catmull-Rom spline *through every*
     control point, then arc length from a dense polyline along that curve.
     This matches the *idea* of Unreal’s `USplineComponent` (smooth between
     knots) much better than chord-only sampling. Unreal’s exact tangents can
     still differ slightly; for strict chord-following use `--curve polyline`.
   - **`polyline`:** straight segments only (legacy); same as the old
     `Spline` / `load_spline_from_map()` behavior.
3. Steps along the curve by fixed arc length (`--step`). At each step:
   - **Position + yaw** come from the curve at `s`, with yaw from a
     **look-ahead** along the curve (`--lookahead`) so the pawn aims where it
     is going next (smaller `--step` and `--lookahead` → more, gentler turns).
4. Optional **pose jitter** (`--jitter-xy`, `--jitter-yaw`, `--seed`) adds small
   random offsets for augmentation; the manifest keeps `nominal_pose` on the
   curve and `pose` as what is sent to Unreal.
5. Sends UnrealCV:

   ```
   vset /object/<pawn>/location <x> <y> <z>
   vset /object/<pawn>/rotation 0 <yaw> 0
   ```

   with a Blueprint fallback (`vbp <pawn> SetActorLocation/Rotation`).
6. Optionally captures `vget /camera/<id>/lit <path>` after `--settle`.
   By default, collection prefers camera `0` (pawn/player view). Use
   `--camera-id` (or `BOXSIM_UNREALCV_CAMERA_ID`) to force another camera.
7. Appends JSONL rows: `curve_mode`, `frame_idx`, `distance_along_path`,
   `pose`, `label`, `label_detail`, `image`, `path_source`, and when jitter is
   on: `nominal_pose`, `jitter`.

## Prerequisites

- BoxSim requirements installed (`pip install -r ../requirements.txt`).
- Unreal Engine running your project with:
  - The **UE5 Path Tape** plugin enabled, and a `PathManagerActor` placed
    in the level that has been **rebuilt from your map JSON** so the spline
    exists in the world.
  - UnrealCV listening on `localhost:9000`.
- A BoxSim map JSON that contains a `"path"` list in world space
  (i.e. `"path_space": "world"`). Produce this from `build.py manual` /
  `build.py screenshot` in BoxSim — the default file is
  [`data/maps/map_w.json`](../data/maps/map_w.json).

## Run it

From the BoxSim repo root:

```bash
# Smoke test — no UnrealCV. Default curve is catmull_rom (smooth through knots).
python data_collection/collect.py --dry-run --run-name run1 --step 25

# Chord-only path (old behavior).
python data_collection/collect.py --dry-run --run-name run1 --curve polyline

# Denser steps + shorter lookahead → more frames, smaller heading changes.
python data_collection/collect.py --dry-run --run-name run1 --step 10 --lookahead 40

# Augmentation: small XY and yaw noise (reproducible with --seed).
python data_collection/collect.py --dry-run --run-name run1 --jitter-xy 2 --jitter-yaw 1.5 --seed 123

# Real run — teleport + lit capture per step.
python data_collection/collect.py --run-name run1 --step 25 --lookahead 75

set UNREALCV_PAWN=BP_MyPlayer_Pawn_C_1
python data_collection/collect.py --run-name run1 --no-capture
```

### CLI flags

| Flag | Default | What |
|---|---|---|
| `--map` | `data/maps/map_w.json` | Map JSON containing the `path` polyline. |
| `--run-name` | `run` | Subfolder under `--output-root` for this dataset. |
| `--output-root` | `data/datasets` | Where runs are written. |
| `--step` | `25.0` | Arc-length step between frames (UE cm). Smaller = more poses. |
| `--lookahead` | `75.0` | Arc-length look-ahead for yaw + turn label (UE cm). |
| `--settle` | `0.12` | Seconds after each teleport before capture. |
| `--turn-threshold` | `15.0` | `abs(delta_yaw)` above which the label becomes `left`/`right`. |
| `--curve` | `catmull_rom` | `catmull_rom` (smooth through points) or `polyline` (chords). |
| `--curve-samples` | `48` | Samples per control segment for arc-length table (catmull only). |
| `--jitter-xy` | `0` | Uniform X and Y offset each in `[-value, +value]` cm. |
| `--jitter-yaw` | `0` | Uniform yaw offset in `[-value, +value]` degrees. |
| `--seed` | none | RNG seed for jitter. |
| `--pawn` | `$UNREALCV_PAWN` → `BP_MyPlayer_Pawn_C_1` | Pawn object name. |
| `--pawn-z` | current pawn Z | World Z (cm) used during teleports. |
| `--camera-id` | auto | Lit camera id (see main README / `capture.py`). |
| `--yaw-offset` | `0.0` | Added to curve tangent yaw before send / log. |
| `--no-capture` | off | Teleport + manifest, skip PNGs. |
| `--dry-run` | off | Skip UnrealCV; write manifest from the curve only. |
| `--debug` | off | Log every UnrealCV request/response. |

## Output layout

```
data/datasets/<run-name>/
  run_config.json        # snapshot of run parameters
  manifest.jsonl         # one JSON object per frame
  images/
    frame_00000.png
    ...
```

Example manifest row (default curve, no jitter):

```json
{
  "schema_version": 1,
  "frame_idx": 0,
  "distance_along_path": 0.0,
  "curve_mode": "catmull_rom",
  "pose": {"x": -206.9, "y": 149.3, "z": 0.0, "yaw_deg": -175.6},
  "label": "forward",
  "label_detail": {
    "delta_yaw_deg": -14.99,
    "yaw_now_deg": -175.63,
    "yaw_ahead_deg": 169.38,
    "s_ahead": 75.0
  },
  "image": "images/frame_00000.png",
  "path_source": "C:\\...\\BoxSim\\data\\maps\\map_w.json"
}
```

With jitter, `pose` is the teleported pose and `nominal_pose` / `jitter`
record the on-curve target and the applied offsets.

## Module layout

- [`spline.py`](spline.py) — `PolylinePath`, `CatmullRomPath`, `SplineSample`,
  `load_path_from_map()`, `load_spline_from_map()` (polyline-only alias).
- [`pawn_control.py`](pawn_control.py) — `teleport_pawn()`.
- [`capture.py`](capture.py) — `resolve_lit_camera_id()`, `capture_lit_image()`.
- [`collect.py`](collect.py) — CLI pipeline.

## How this ties to UE5PathTapePrototype

In the plugin, `PathManagerActor::BuildSplineFromMetadata` pushes the same
JSON points into `USplineComponent::SetSplinePoints`; Unreal then evaluates a
smooth spline (with engine default tangents). This repo cannot call
`GetLocationAtDistanceAlongSpline` from Python without extra Blueprint hooks,
so **`catmull_rom`** is a practical stand-in: it passes through every JSON
knot and rounds corners smoothly, giving **many intermediate samples** when
you use a small `--step`. Use **`polyline`** only when you explicitly want
motion constrained to straight chords between knots.

## Troubleshooting

- **Pawn doesn’t move** — set `UNREALCV_PAWN` to the **object** name; use
  `python main.py --list-objects`.
- **Pawn falls through the floor** — `--pawn-z` or rely on read Z at connect.
- **Wrong heading** — `--yaw-offset` or tune `--lookahead` / `--step`.
- **`path_space` error** — re-export with `"path_space": "world"`.
- **Curve vs tape mismatch** — try `--curve polyline` to isolate; or add a
  small Blueprint function on `PathManagerActor` that returns world XY at
  distance `s` and call it via `vbp` (future improvement).
