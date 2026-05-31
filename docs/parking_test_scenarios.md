# Memory Parking Test Map

HPASim currently uses one complete OpenDRIVE parking lot map:

| Scenario | Target | Purpose |
| --- | --- | --- |
| `parking_lot_full` | `central_upper_angled_row_13` | End-to-end memory parking in a complete lot with entry, exit, drive aisles, mixed parking rows, occupied vehicles, islands, crossings, speed bumps, and a target bay. |

## Map Content

- Three longitudinal drive aisles: entry, memory, and exit.
- Three cross aisles connecting the longitudinal drives.
- Mixed parking rows with dense perpendicular spaces, angled spaces, and
  boundary-side parallel spaces.
- Occupied vehicles, accessible spaces, charging spaces, reserved spaces, and
  one target memory-parking bay.
- Boundary walls, entry and exit gates, landscape islands, pedestrian crossings,
  speed bumps, a delivery vehicle obstacle, and a work-zone obstacle.

## Modeling Notes

- The map is generated from scratch as a single integrated parking lot, not as a
  collection of separate maneuver scenes.
- Roads are straight OpenDRIVE reference lines for low-speed simulation.
- Parking spaces and static features are encoded as OpenDRIVE `object` elements
  with rectangular `outline` polygons.
- The matplotlib renderer draws geometry only. It intentionally does not draw
  text labels on the map.

## Commands

Generate the map:

```powershell
uv run python scripts/generate_maps.py
```

Render the map:

```powershell
uv run python hpasim/plot_opendrive.py
```
