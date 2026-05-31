# HPASim

[中文说明](README.zh-CN.md)

HPASim is a memory parking simulation scaffold. The current workspace focuses on
one complete OpenDRIVE underground-style parking lot map, a kinematic vehicle
model, route planning, static matplotlib rendering, and an interactive browser
viewer.

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

## Interactive Viewer

Run the local browser viewer:

```powershell
uv run python scripts/serve_viewer.py
```

Then open `http://127.0.0.1:8000`. The viewer uses Canvas for fast interaction:
wheel to zoom, drag to pan, and click a parking space to request a route from
the ego start pose to that space.

## Vehicle Model

The vehicle model is a front-steering kinematic bicycle model with state
`x, y, yaw, v` and control input `steer, acceleration`.

## Route Planning

The planner loads the generated OpenDRIVE map, builds a grid occupancy map from
drive aisles and static obstacles, and runs A* to produce a route from the ego
start pose toward a selected parking space.

```powershell
uv run python scripts/plan_route.py
```

Run the tests:

```powershell
uv run python -m unittest discover -s tests
```

## Key Files

- `hpasim/parking_scenarios.py`: scenario definition and OpenDRIVE generation.
- `hpasim/plot_opendrive.py`: matplotlib renderer.
- `hpasim/vehicle.py`: front-steering kinematic vehicle model.
- `hpasim/planner.py`: grid route planner for OpenDRIVE maps.
- `hpasim/map_payload.py`: JSON-ready map data for the interactive viewer.
- `scripts/serve_viewer.py`: local HTTP server for the Canvas viewer.
- `viewer/index.html`: interactive Canvas viewer.
