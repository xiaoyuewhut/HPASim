from pathlib import Path
import unittest

from hpasim.planner import GridPlannerConfig, GridRoutePlanner, load_opendrive_map


MAP_PATH = Path("maps/opendrive/parking_lot_full.xodr")


class GridRoutePlannerTest(unittest.TestCase):
    def test_loads_map_objects(self) -> None:
        opendrive_map = load_opendrive_map(MAP_PATH)

        self.assertGreaterEqual(len(opendrive_map.road_polygons), 1)
        self.assertEqual(opendrive_map.find_object("central_upper_angled_row_13").subtype, "target")

    def test_plans_to_target_space(self) -> None:
        opendrive_map = load_opendrive_map(MAP_PATH)
        planner = GridRoutePlanner(opendrive_map, GridPlannerConfig(resolution=1.0, obstacle_padding=0.2))

        path = planner.plan_to_object((8.0, -1.7), "central_upper_angled_row_13")

        self.assertGreater(len(path), 2)
        self.assertAlmostEqual(path[0][0], 8.0, delta=1.0)
        self.assertAlmostEqual(path[0][1], -1.7, delta=1.0)
        target = opendrive_map.find_object("central_upper_angled_row_13")
        self.assertAlmostEqual(path[-1][0], target.center[0], delta=4.0)
        self.assertAlmostEqual(path[-1][1], target.center[1], delta=5.0)


if __name__ == "__main__":
    unittest.main()
