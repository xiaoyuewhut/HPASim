import unittest
import math

from hpasim.map_payload import load_default_map
from hpasim.planner import GridPlannerConfig, GridRoutePlanner
from hpasim.trajectory import TrajectoryPlanner, TrajectoryPlannerConfig
from hpasim.vehicle import normalize_angle


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

    def test_reverse_segment_has_real_backward_motion(self) -> None:
        opendrive_map = load_default_map()
        route = GridRoutePlanner(
            opendrive_map,
            GridPlannerConfig(resolution=1.0, obstacle_padding=0.2),
        ).plan_route_to_object((8.0, -1.7), "central_upper_angled_row_13")

        trajectory = TrajectoryPlanner(TrajectoryPlannerConfig(spacing=0.5)).plan(route)
        first_reverse = next(index for index, point in enumerate(trajectory.points) if point.gear == "reverse")

        self.assertTrue(any(point.v < 0.0 for point in trajectory.points[first_reverse:]))

        reverse_direction_errors = []
        for point, next_point in zip(trajectory.points[first_reverse:], trajectory.points[first_reverse + 1 :]):
            if math.hypot(next_point.x - point.x, next_point.y - point.y) < 1e-6:
                continue
            motion_yaw = math.atan2(next_point.y - point.y, next_point.x - point.x)
            reverse_direction_errors.append(abs(normalize_angle(motion_yaw - point.yaw - math.pi)))

        self.assertTrue(reverse_direction_errors)
        self.assertLess(max(reverse_direction_errors), math.radians(5.0))
        self.assertAlmostEqual(trajectory.points[-1].yaw, route.target_yaw, delta=math.radians(5.0))

    def test_local_trajectory_smooths_global_route_near_parking_space(self) -> None:
        opendrive_map = load_default_map()
        route = GridRoutePlanner(
            opendrive_map,
            GridPlannerConfig(resolution=1.0, obstacle_padding=0.2),
        ).plan_route_to_object((8.0, -1.7), "central_upper_angled_row_13")

        trajectory = TrajectoryPlanner(TrajectoryPlannerConfig(spacing=0.5)).plan(route)
        raw_reverse = route.segments[-1].points
        first_reverse = next(index for index, point in enumerate(trajectory.points) if point.gear == "reverse")
        reverse_deviation = [
            _distance_to_polyline((point.x, point.y), raw_reverse)
            for point in trajectory.points[first_reverse:]
        ]

        self.assertGreater(max(reverse_deviation), 0.15)


def _distance_to_polyline(point: tuple[float, float], polyline: tuple[tuple[float, float], ...]) -> float:
    return min(_distance_to_segment(point, start, end) for start, end in zip(polyline, polyline[1:]))


def _distance_to_segment(point: tuple[float, float], start: tuple[float, float], end: tuple[float, float]) -> float:
    px, py = point
    sx, sy = start
    ex, ey = end
    dx = ex - sx
    dy = ey - sy
    length_sq = dx * dx + dy * dy
    if length_sq == 0.0:
        return math.hypot(px - sx, py - sy)
    t = max(0.0, min(1.0, ((px - sx) * dx + (py - sy) * dy) / length_sq))
    projection = (sx + t * dx, sy + t * dy)
    return math.hypot(px - projection[0], py - projection[1])


if __name__ == "__main__":
    unittest.main()
