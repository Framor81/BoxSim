# BoxSim

BoxSim connects **Python** to **Unreal Engine 5** through **UnrealCV**. You use it to define a driving path on a track, record labeled camera frames along that path, train a small **left / right / forward** image classifier, and run that model back in the sim.

The sibling repo **`UE5PathTapePrototype`** is an Unreal plugin: it loads path JSON in the editor, draws a spline (optional tape/background), and **exports the path in true UE world coordinates**. BoxSim and Path Tape share the same JSON shape (`path`, `path_points`, or `world_points`).

---

## What this repo does

| Area | What | Entry point |
|------|------|-------------|
| **Connect to UE** | Pose polling, object list, debug UnrealCV | `main.py`, `agent.py` |
| **Drive manually** | Send W/A/S/D sequences to the pawn | `drive.py` |
| **Build maps** | Top-down capture or draw obstacles + **path**; save PNG + JSON | `build.py` |
| **World path** | Confirm path in-level; export UE cm coordinates | **UE5 Path Tape** (`PathManagerActor`) |
| **Collect data** | Teleport pawn along path; save images + `manifest.jsonl` | `data_collection/collect.py` |
| **Train** | Fine-tune ResNet on collected frames | `training.py` |
| **Eval** | Closed-loop driving + metrics on a map | `eval.py` |

**Typical end-to-end work:** map/trace path → Path Tape world export → collect → train → eval.

**Data layout (after you run things):**

```
data/maps/          # map.png + .json + *_w.json (world path)
data/datasets/      # collection runs (manifest + images/)
models/             # trained classifier (.pkl)
data/evals/         # eval logs and step images
config/boxsim.json  # capture alignment (gitignored; copy from example)
```

---

## Set up

1. **Python 3.10+** and dependencies:
   ```bash
   pip install -r requirements.txt
   ```

2. **Unreal project** with UnrealCV listening on **`localhost:9000`**. Start the editor or packaged game before any BoxSim command that talks to UE.

3. **UE5 Path Tape** — Copy `Plugins/UE5PathTapePrototype/` from the sibling repo into your project’s `Plugins/`, enable **UE5 Path Tape** in the editor, rebuild. See that repo’s README for actor details.

4. **Pawn name** (required for drive, collection, eval):
   ```bash
   python main.py --list-objects
   export UNREALCV_PAWN="<object_name>"    # not the display name
   ```

5. **Screenshot maps only** — Align the top-down capture with your level:
   ```bash
   cp config/boxsim.example.json config/boxsim.json
   ```
   Adjust if the lit image and live robot overlay don’t match. Details: [`builder/capture_config.py`](builder/capture_config.py).

---

## How to Run

Unreal must be running for every step below except dry-run collection and `build.py frompng`.

### End-to-end pipeline (recommended first read)

```
1. Trace path in BoxSim          →  map_w.json (approx world XY)
2. Load JSON in Path Tape (UE)   →  spline visible on track
3. Export world points (UE)      →  path_world_points.json (ground truth)
4. collect.py                    →  data/datasets/<run>/
5. training.py                   →  models/turn_classifier.pkl
6. eval.py                       →  data/evals/<run>/
```

---

### A. Build a map (path + optional obstacles)

Open the pygame map editor, draw geometry, save with **Ctrl+S**.

| Mode | Command | Output | Notes |
|------|---------|--------|--------|
| **Screenshot** | `python build.py screenshot` | `data/maps/map.png`, `map.json`, **`map_w.json`** | Grabs ortho lit image from UE; best when `config/boxsim.json` matches your level |
| **Manual grid** | `python build.py manual` | `data/maps/manual_map*` | Blank grid from config bounds |
| **Manual + robot** | `python build.py manual --unreal` | same | Live pose overlay while tracing |
| **Floor plan PNG** | `python build.py frompng 1.png --width-in 144 --length-in 96` | `outline_trace.json` (inches) | No `*_w.json`; use Path Tape + export, or `scripts/normalize_map_json.py` |

**Editor basics:** `E` = path tool (click waypoints in order; close on first point or right-click) · `1`/`2`/`3` = poly / brush / box · `A`/`S`/`D` = obstacle / drivable / cut · `W` = goal · **Ctrl+S** save · **Ctrl+Z** undo.

For **data collection**, you need a JSON whose polyline is in **UE world cm**. Easiest source: the **`*_w.json`** file from screenshot/manual save. Collection only needs the path polyline, not obstacles.

---

### B. Path Tape — world coordinates in UE

Use this when the path must match the **actual level** (or you started from plan/inch JSON).

1. In UE, place **`PathManagerActor`**.
2. Set **Json File Path** to your BoxSim file (e.g. `data/maps/map_w.json` or `map.json`).
3. **Rebuild From Json** — spline (and optional tape) appear in the level.
4. If scale/position is wrong: set **`"path_space": "world"`** in JSON, or use the actor’s **World XY** override (coordinates already in UE cm).
5. **Export World Points To Json** — writes e.g. `Saved/PathExports/path_world_points.json` with **`world_points`** in world space.

That export is what you pass to **`collect.py --map`**. You can also merge plan + world export for outline tracks: `python scripts/normalize_map_json.py 4` → `outline_trace4_w.json`.

BoxSim collection accepts either **`"path"`** or **`"world_points"`** (Path Tape export format).

---

### C. Collect training data

Moves the pawn along the path in steps, labels each frame **forward / left / right**, optionally saves lit PNGs from the pawn camera.

**Dry run** (validates path, no UE):

```bash
python data_collection/collect.py \
  --dry-run \
  --map data/maps/map_w.json \
  --run-name run1
```

**Live collection** (UE running, pawn on drivable surface):

```bash
python data_collection/collect.py \
  --map Saved/PathExports/path_world_points.json \
  --run-name run1 \
  --step 25 \
  --lookahead 75
```

**Output:** `data/datasets/run1/manifest.jsonl`, `run_config.json`, `images/frame_*.png`.

| Flag | Role |
|------|------|
| `--map` | JSON with world-space path (default `data/maps/map_w.json`) |
| `--run-name` | Folder under `data/datasets/` |
| `--step` | cm between frames along the path (smaller → more data) |
| `--lookahead` | cm ahead for heading and turn labels |
| `--no-capture` | Teleport + manifest only (no images) |
| `--curve polyline` | Chord path; default is smooth `catmull_rom` |

If the pawn doesn’t move: check `UNREALCV_PAWN`. If it falls: `--pawn-z`. If heading is wrong: `--yaw-offset` or tune `--lookahead` / `--step`.

Full CLI: [`data_collection/README.md`](data_collection/README.md).

---

### D. Train the policy

```bash
python training.py --data-root data/datasets --run-glob "run1"
```

Default: ResNet18, classes `left,right,forward`, writes e.g. `models/turn_classifier.pkl`. Use `--run-glob "run*"` to combine multiple collection runs.

---

### E. Evaluate in the sim

Runs the trained model closed-loop on the same map path, logs centerline error and saves step images.

```bash
python eval.py \
  --model models/turn_classifier.pkl \
  --map data/maps/map_w.json \
  --finish-mode lap
```

For **closed loops**, prefer **`--finish-mode lap`** (not plain distance). Outputs under `data/evals/<run-name>/`.

---

### F. Other Unreal tools

| Task | Command |
|------|---------|
| Stream pose at 5 Hz | `python main.py` |
| List UnrealCV objects | `python main.py --list-objects` |
| Debug UnrealCV traffic | `python main.py --debug` |
| Run keyboard path in `drive.py` | Edit `PATH` in `drive.py`, then `python drive.py` |

---

## Info

**Two repos, one pipeline** — BoxSim traces and stores maps/paths; Path Tape places the path in UE and exports authoritative world XY. Collection and eval consume that world JSON.

**Config** — `config/boxsim.json` controls screenshot bounds, ortho size, lit rotation/flips, and pose overlay. Env vars override the file (`BOXSIM_*`); see `builder/capture_config.py`.

**Deeper docs** — Collection: [`data_collection/README.md`](data_collection/README.md). Path Tape plugin: **UE5PathTapePrototype** README.

**Helper scripts** (optional) — `scripts/normalize_map_json.py` (merge outline + world export), `scripts/densify_map_path.py`, `scripts/emit_path_snippet.py`, test runners under `scripts/*.ps1`.
