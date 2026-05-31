"""Plan a route on the generated OpenDRIVE parking map."""

from pathlib import Path
import sys


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from hpasim.parking_scenarios import SCENARIOS
from hpasim.planner import GridPlannerConfig, GridRoutePlanner, load_opendrive_map


def main() -> None:
    scenario = SCENARIOS[0]
    map_path = PROJECT_ROOT / "maps" / "opendrive" / f"{scenario.key}.xodr"
    opendrive_map = load_opendrive_map(map_path)
    planner = GridRoutePlanner(opendrive_map, GridPlannerConfig(resolution=1.0, obstacle_padding=0.2))
    path = planner.plan_to_object((scenario.ego_start[0], scenario.ego_start[1]), scenario.target_parking_space)
    for x, y in path:
        print(f"{x:.3f},{y:.3f}")


if __name__ == "__main__":
    main()
