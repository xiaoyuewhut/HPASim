# HPASim

[中文说明](README.zh-CN.md)

HPASim is a memory parking simulation scaffold. The current workspace focuses on
one complete OpenDRIVE parking lot map and a matplotlib renderer for quick visual
inspection.

## Current Map

The active scenario is `parking_lot_full`.

- Target parking space: `central_upper_angled_row_13`
- OpenDRIVE output: `maps/opendrive/parking_lot_full.xodr`
- Preview output: `outputs/parking_lot_full.png`

The map includes entry and exit drives, three longitudinal drive aisles, three
cross aisles, dense perpendicular spaces, angled spaces, boundary-side parallel
spaces, occupied vehicles, charging spaces, accessible spaces, reserved spaces,
pedestrian walkways, entry and exit ramps, barrier gates, speed bumps, and
static obstacles.

## Generate The Map

```powershell
uv run python scripts/generate_maps.py
```

## Render The Map

```powershell
uv run python hpasim/plot_opendrive.py
```

The renderer intentionally draws geometry only: no object labels, no title, and
no axis text.

## Vehicle Model

The vehicle model is a front-steering kinematic bicycle model with state
`x, y, yaw, v` and control input `steer, acceleration`.

Run the tests:

```powershell
uv run python -m unittest discover -s tests
```

## Key Files

- `hpasim/parking_scenarios.py`: scenario definition and OpenDRIVE generation.
- `hpasim/plot_opendrive.py`: matplotlib renderer.
- `hpasim/vehicle.py`: front-steering kinematic vehicle model.
- `docs/parking_test_scenarios.md`: detailed map notes.
