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

Connects to UnrealCV (port 9000), prints BP_MyPlayer_Pawn_C pose at 5 Hz. Ctrl+C to stop.

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
