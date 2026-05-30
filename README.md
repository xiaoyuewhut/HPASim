# HPASim

Memory parking simulation scaffolding with generated OpenDRIVE parking maps and
matplotlib visualization support.

## Generate Maps

```powershell
python scripts/generate_maps.py
```

Generated maps are written to `maps/opendrive/*.xodr`.

## Plot A Map

```powershell
python scripts/plot_map.py maps/opendrive/parallel_curb_slot.xodr --output outputs/parallel_curb_slot.png
```

The initial scenario definitions are documented in
`docs/parking_test_scenarios.md`.
