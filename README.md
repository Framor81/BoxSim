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

**“Pose (unavailable)” but connection works?** UnrealCV is connected but the pawn name may not match. You need a **pawn in the level** (your default player pawn or one you placed).

1. **List object names** UnrealCV can see:
   ```bash
   python main.py --list-objects
   ```
2. **Use that name** (e.g. if you see `BP_MyPlayer_Pawn_0` or `PlayerPawn_0`):
   ```bash
   UNREALCV_PAWN=BP_MyPlayer_Pawn_0 python main.py
   ```
3. **Debug** to see the raw response when pose fails:
   ```bash
   python main.py --debug
   ```

In Unreal: your **Game Mode** should use a Blueprint that has a **Default Pawn Class** set to your pawn Blueprint (e.g. `BP_MyPlayer_Pawn`). The script uses the **class** name with `_C` (e.g. `BP_MyPlayer_Pawn_C`) or the **instance** name from `--list-objects`.

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
