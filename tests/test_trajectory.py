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

    def test_speed_profile_respects_acceleration_and_parking_creep(self) -> None:
        opendrive_map = load_default_map()
        route = GridRoutePlanner(
            opendrive_map,
            GridPlannerConfig(resolution=1.0, obstacle_padding=0.2),
        ).plan_route_to_object((8.0, -1.7), "central_upper_angled_row_13")
        config = TrajectoryPlannerConfig(
            spacing=0.5,
            max_acceleration=0.8,
            max_deceleration=1.0,
            parking_creep_distance=2.0,
            parking_creep_speed=0.25,
        )

        trajectory = TrajectoryPlanner(config).plan(route)

        self.assertAlmostEqual(trajectory.points[0].v, 0.0)
        self.assertAlmostEqual(trajectory.points[-1].v, 0.0)
        for point in trajectory.points:
            if point.gear == "forward":
                self.assertGreaterEqual(point.v, -1e-9)
                self.assertLessEqual(point.v, config.forward_speed + 1e-9)
            else:
                self.assertLessEqual(point.v, 1e-9)
                self.assertGreaterEqual(point.v, config.reverse_speed - 1e-9)

        for point, next_point in zip(trajectory.points, trajectory.points[1:]):
            if next_point.t <= point.t:
                continue
            speed_change = (abs(next_point.v) - abs(point.v)) / (next_point.t - point.t)
            self.assertLessEqual(speed_change, config.max_acceleration + 1e-6)
            self.assertGreaterEqual(speed_change, -config.max_deceleration - 1e-6)

        reverse_points = [point for point in trajectory.points if point.gear == "reverse"]
        self.assertLess(max(abs(point.v) for point in reverse_points[-5:]), abs(config.reverse_speed))
        self.assertLessEqual(abs(reverse_points[-2].v), config.parking_creep_speed + 0.2)

    def test_curve_speed_limit_slows_tight_turns(self) -> None:
        opendrive_map = load_default_map()
        route = GridRoutePlanner(
            opendrive_map,
            GridPlannerConfig(resolution=1.0, obstacle_padding=0.2),
        ).plan_route_to_object((8.0, -1.7), "central_upper_angled_row_13")
        config = TrajectoryPlannerConfig(spacing=0.5, lateral_acceleration_limit=0.45)

        trajectory = TrajectoryPlanner(config).plan(route)
        curved_points = [
            point
            for point in trajectory.points
            if point.gear == "forward" and abs(point.curvature) > 0.2 and point.v > 0.0
        ]

        self.assertTrue(curved_points)
        self.assertLess(min(point.v for point in curved_points), config.forward_speed)


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
