# BoxSim

Unreal Engine 5 + UnrealCV: pose, drive, map building.

```bash
pip install -r requirements.txt
```

| Command | What |
|---------|------|
| `python main.py` | Pose at 5 Hz. Ctrl+C stop. |
| `python main.py --list-objects` | List object names (use as `UNREALCV_PAWN`) |
| `python main.py --debug` | Raw UnrealCV responses |
| `python drive.py` | Run PATH in drive.py (W/A/S/D sent to Unreal) |
| `python build.py screenshot` | Top-down from Unreal, annotate, save |
| `python build.py manual` | Blank canvas, draw obstacles |

Env: `UNREALCV_PAWN` if your pawn object name differs (use object name, not display name).

Map controls: 1=Polygon 2=Brush 3=Box S=Save Esc=Close
