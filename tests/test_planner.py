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

    def test_plans_forward_and_reverse_segments_to_parking_space(self) -> None:
        opendrive_map = load_opendrive_map(MAP_PATH)
        planner = GridRoutePlanner(opendrive_map, GridPlannerConfig(resolution=1.0, obstacle_padding=0.2))

        route = planner.plan_route_to_object((8.0, -1.7), "central_upper_angled_row_13")

        self.assertEqual([segment.gear for segment in route.segments], ["forward", "reverse"])
        self.assertIsNotNone(route.target_yaw)
        self.assertGreater(len(route.segments[0].points), 2)
        self.assertGreater(len(route.segments[1].points), 2)
        target = opendrive_map.find_object("central_upper_angled_row_13")
        self.assertAlmostEqual(route.points[-1][0], target.center[0], delta=0.1)
        self.assertAlmostEqual(route.points[-1][1], target.center[1], delta=0.1)

    def test_forward_route_does_not_cut_through_parking_spaces(self) -> None:
        opendrive_map = load_opendrive_map(MAP_PATH)
        planner = GridRoutePlanner(opendrive_map, GridPlannerConfig(resolution=1.0, obstacle_padding=0.2))

        parking_polygons = tuple(obj.polygon for obj in opendrive_map.objects if obj.object_type == "parkingSpace")

        for target in (
            "central_upper_angled_row_17",
            "north_perpendicular_row_17",
            "south_perpendicular_row_17",
        ):
            with self.subTest(target=target):
                route = planner.plan_route_to_object((8.0, -1.7), target)
                non_target_parking_polygons = tuple(
                    obj.polygon
                    for obj in opendrive_map.objects
                    if obj.object_type == "parkingSpace" and obj.name != target
                )
                for point in _sample_polyline(route.segments[0].points, step=0.1):
                    self.assertFalse(
                        any(_point_in_polygon(point, polygon) for polygon in parking_polygons),
                        f"forward route cuts through a parking space at {point}",
                    )
                for segment in route.segments:
                    for point in _sample_polyline(segment.points, step=0.1):
                        self.assertFalse(
                            any(_point_in_polygon(point, polygon) for polygon in non_target_parking_polygons),
                            f"route to {target} cuts through a non-target parking space at {point}",
                        )


def _sample_polyline(points: tuple[tuple[float, float], ...], step: float) -> list[tuple[float, float]]:
    samples = [points[0]]
    for start, end in zip(points, points[1:]):
        distance = ((end[0] - start[0]) ** 2 + (end[1] - start[1]) ** 2) ** 0.5
        count = max(1, int(distance / step))
        samples.extend(
            (
                start[0] + (end[0] - start[0]) * index / count,
                start[1] + (end[1] - start[1]) * index / count,
            )
            for index in range(1, count + 1)
        )
    return samples


def _point_in_polygon(point: tuple[float, float], polygon: tuple[tuple[float, float], ...]) -> bool:
    x, y = point
    inside = False
    j = len(polygon) - 1
    for i, pi in enumerate(polygon):
        xi, yi = pi
        xj, yj = polygon[j]
        if (yi > y) != (yj > y) and x < (xj - xi) * (y - yi) / (yj - yi) + xi:
            inside = not inside
        j = i
    return inside


if __name__ == "__main__":
    unittest.main()
