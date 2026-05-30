# HPASim

Memory parking simulation scaffolding with one generated OpenDRIVE parking lot
map and matplotlib visualization support.

## Generate Maps

```powershell
python scripts/generate_maps.py
```

The generated map is written to `maps/opendrive/parking_lot_full.xodr`.

## Plot A Map

```powershell
uv run python hpasim/plot_opendrive.py
```

The initial scenario definitions are documented in
`docs/parking_test_scenarios.md`.
