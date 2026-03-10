# UnrealCV Environment Mapper (MVP)

Python tool for Unreal Engine 5.6 + UnrealCV: pawn pose and map building.

## Setup

```bash
pip install -r requirements.txt
```

## Pose polling

```bash
python main.py
```

Connects to UnrealCV (port 9000), prints pawn pose at 5 Hz. Ctrl+C to stop.

**“Pose (unavailable)” but connection works?** You must use the **object name**, not the display name. In Unreal the Outliner may show a display name (e.g. “BP_MyPlayer_Pawn”); UnrealCV expects the **object name** (e.g. `BP_MyPlayer_Pawn_C_1`). The default pawn name is `BP_MyPlayer_Pawn_C_1`; override with `UNREALCV_PAWN` if yours differs.

1. **List object names** UnrealCV can see:
   ```bash
   python main.py --list-objects
   ```
   Use one of the printed names (object name) as `UNREALCV_PAWN`.
2. **Set the pawn** (use the exact object name from the list):
   ```bash
   UNREALCV_PAWN=BP_MyPlayer_Pawn_C_1 python main.py
   ```
3. **Debug** to see every command sent and raw response:
   ```bash
   python main.py --debug
   ```

In Unreal: set your **Game Mode** → **Default Pawn Class** to your pawn Blueprint. The script uses the **object name** (e.g. `BP_MyPlayer_Pawn_C_1`), not the Blueprint display name.

## Map builder

```bash
python build.py screenshot   # Top-down from Unreal, then annotate
python build.py manual      # Blank canvas, draw boxes yourself
```

**Controls:** 1=Polygon, 2=Brush, 3=Box, S=Save, Esc=Close

## Layout

| Path | Role |
|------|------|
| `main.py` | Pose polling entry |
| `build.py` | Map builder entry |
| `agent.py` | UnrealCV client, pawn pose |
| `map_utils.py` | Coords (world↔pixel) + map load/save |
| `builder/viewer.py` | MapViewer (pygame UI) |
| `builder/builders.py` | ScreenshotMapBuilder, ManualMapBuilder (two flows) |
| `data/screenshots/`, `data/maps/` | Output dirs |
