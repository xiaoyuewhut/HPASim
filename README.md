# HPASim

Memory parking simulation scaffolding with generated OpenDRIVE parking maps and
matplotlib visualization support.

## Generate Maps

```powershell
python scripts/generate_maps.py
```

Generated maps are written to `maps/opendrive/*.xodr`, including the larger
`complex_parking_lot.xodr` integration scenario.

## Plot A Map

```powershell
python scripts/plot_map.py maps/opendrive/parallel_curb_slot.xodr --output outputs/parallel_curb_slot.png
```

Or render every map:

```powershell
uv run python hpasim/plot_opendrive.py
```

The initial scenario definitions are documented in
`docs/parking_test_scenarios.md`.
