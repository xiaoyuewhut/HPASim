import unittest

from hpasim.map_payload import load_default_map
from hpasim.planner import GridPlannerConfig, GridRoutePlanner
from hpasim.trajectory import TrajectoryPlanner, TrajectoryPlannerConfig


class TrajectoryPlannerTest(unittest.TestCase):
    def test_generates_forward_and_reverse_reference_points(self) -> None:
        opendrive_map = load_default_map()
        route = GridRoutePlanner(
            opendrive_map,
            GridPlannerConfig(resolution=1.0, obstacle_padding=0.2),
        ).plan_route_to_object((8.0, -1.7), "central_upper_angled_row_13")

        trajectory = TrajectoryPlanner(TrajectoryPlannerConfig(spacing=0.5)).plan(route)

        self.assertGreater(len(trajectory), 20)
        gears = {point.gear for point in trajectory.points}
        self.assertEqual(gears, {"forward", "reverse"})
        self.assertTrue(any(point.v > 0.0 for point in trajectory.points))
        self.assertTrue(any(point.v < 0.0 for point in trajectory.points))
        self.assertAlmostEqual(trajectory.points[0].t, 0.0)
        self.assertGreater(trajectory.points[-1].t, trajectory.points[0].t)


if __name__ == "__main__":
    unittest.main()
