# Memory Parking Test Scenarios

The initial HPASim scenario set focuses on low-speed memory parking in simple,
repeatable layouts. Each scenario is generated as OpenDRIVE and can be rendered
with the matplotlib plotter.

| Scenario | Target | Purpose |
| --- | --- | --- |
| `parallel_curb_slot` | `target_parallel_01` | Urban curbside parallel parking with parked vehicles before and after the target slot. |
| `perpendicular_bay_row` | `target_perpendicular_01` | Parking-lot aisle with perpendicular bays on both sides and one occupied neighbor. |
| `angled_60deg_bay` | `target_angled_01` | Forward-search parking in an angled bay layout with a constrained exit side. |
| `narrow_garage_slot` | `target_garage_01` | Memory parking into a tight private garage-like slot bounded by walls. |

## Map Modeling Notes

- Roads are straight OpenDRIVE reference lines with lane widths suitable for
  low-speed parking simulation.
- Parking spaces, occupied vehicles, curbs, walls, wheel stops, and barriers are
  encoded as OpenDRIVE `object` elements with rectangular `outline` polygons.
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
