# Memory Parking Test Scenarios

The HPASim scenario set includes compact maneuver cases and one larger parking
lot map. Each scenario is generated as OpenDRIVE and can be rendered with the
matplotlib plotter.

| Scenario | Target | Purpose |
| --- | --- | --- |
| `parallel_curb_slot` | `target_parallel_01` | Urban curbside parallel parking with parked vehicles before and after the target slot. |
| `perpendicular_bay_row` | `target_perpendicular_01` | Parking-lot aisle with perpendicular bays on both sides and one occupied neighbor. |
| `angled_60deg_bay` | `target_angled_01` | Forward-search parking in an angled bay layout with a constrained exit side. |
| `narrow_garage_slot` | `target_garage_01` | Memory parking into a tight private garage-like slot bounded by walls. |
| `complex_parking_lot` | `central_north_bay_07` | Full parking lot with entry and exit drives, three longitudinal aisles, cross aisles, dense parking rows, occupied vehicles, pedestrian crossings, islands, speed bumps, and a marked memory-parking target. |

## Complex Parking Lot

`complex_parking_lot.xodr` is the main integration scenario. It models:

- South entry drive, central memory drive, north exit drive, and three cross
  aisles.
- Six rows of perpendicular parking spaces with occupied, reserved, charging,
  accessible, and target spaces.
- Boundary walls, gate openings, landscape islands, speed bumps, crosswalks,
  a delivery-van obstacle, and a maintenance-cone obstacle.
- Target parking space `central_north_bay_07`, intended for end-to-end memory
  parking tests after route recording or localization replay is added.

## Map Modeling Notes

- Roads are straight OpenDRIVE reference lines with lane widths suitable for
  low-speed parking simulation.
- Parking spaces, occupied vehicles, curbs, walls, wheel stops, and barriers are
  encoded as OpenDRIVE `object` elements with rectangular `outline` polygons.
- Multi-aisle maps use multiple OpenDRIVE `road` elements. Objects can be tied
  to a specific road reference line through the generator's `road_id` field.
- Target spaces are marked with `subtype="target"` and a light-blue `fill`
  attribute used by the matplotlib renderer.
- Ego starting poses are stored in the Python scenario definitions as
  `(s, t, heading)` in the road-local frame.

## Commands

Generate all maps:

```powershell
python scripts/generate_maps.py
```

Render one map:

```powershell
python scripts/plot_map.py maps/opendrive/parallel_curb_slot.xodr --output outputs/parallel_curb_slot.png
```

Render every generated map:

```powershell
uv run python hpasim/plot_opendrive.py
```
