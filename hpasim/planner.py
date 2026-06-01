"""Grid-based route planning on HPASim OpenDRIVE maps."""

from __future__ import annotations

from dataclasses import dataclass
from heapq import heappop, heappush
from pathlib import Path
import math
import xml.etree.ElementTree as ET


Point = tuple[float, float]
GridIndex = tuple[int, int]


@dataclass(frozen=True)
class RoadFrame:
    x: float
    y: float
    heading: float
    length: float
    left_width: float
    right_width: float

    def st_to_xy(self, s: float, t: float) -> Point:
        cos_h = math.cos(self.heading)
        sin_h = math.sin(self.heading)
        return (
            self.x + s * cos_h - t * sin_h,
            self.y + s * sin_h + t * cos_h,
        )


@dataclass(frozen=True)
class RoadGuide:
    start: Point
    end: Point
    heading: float
    lane_width: float


@dataclass(frozen=True)
class MapObject:
    name: str
    object_type: str
    subtype: str | None
    polygon: tuple[Point, ...]
    center: Point
    heading: float
    length: float
    width: float
    fill: str | None


@dataclass(frozen=True)
class OpenDriveMap:
    road_polygons: tuple[tuple[Point, ...], ...]
    road_guides: tuple[RoadGuide, ...]
    objects: tuple[MapObject, ...]

    def bounds(self, margin: float = 1.0) -> tuple[float, float, float, float]:
        points = [point for polygon in self.road_polygons for point in polygon]
        points.extend(point for obj in self.objects for point in obj.polygon)
        min_x = min(point[0] for point in points) - margin
        max_x = max(point[0] for point in points) + margin
        min_y = min(point[1] for point in points) - margin
        max_y = max(point[1] for point in points) + margin
        return min_x, max_x, min_y, max_y

    def find_object(self, name: str) -> MapObject:
        for obj in self.objects:
            if obj.name == name:
                return obj
        raise KeyError(f"object not found: {name}")


@dataclass(frozen=True)
class GridPlannerConfig:
    resolution: float = 0.5
    obstacle_padding: float = 0.4
    right_lane_weight: float = 0.7
    reverse_staging_distance: float = 4.5

    def __post_init__(self) -> None:
        if self.resolution <= 0.0:
            raise ValueError("resolution must be positive")
        if self.obstacle_padding < 0.0:
            raise ValueError("obstacle_padding must be non-negative")
        if self.right_lane_weight < 0.0:
            raise ValueError("right_lane_weight must be non-negative")
        if self.reverse_staging_distance <= 0.0:
            raise ValueError("reverse_staging_distance must be positive")


@dataclass(frozen=True)
class PathSegment:
    gear: str
    points: tuple[Point, ...]


@dataclass(frozen=True)
class PlannedRoute:
    target: MapObject
    target_yaw: float | None
    segments: tuple[PathSegment, ...]

    @property
    def points(self) -> list[Point]:
        points: list[Point] = []
        for segment in self.segments:
            if points and segment.points and points[-1] == segment.points[0]:
                points.extend(segment.points[1:])
            else:
                points.extend(segment.points)
        return points


@dataclass(frozen=True)
class GridRoutePlanner:
    opendrive_map: OpenDriveMap
    config: GridPlannerConfig = GridPlannerConfig()

    def plan(self, start: Point, goal: Point) -> list[Point]:
        grid = OccupancyGrid.from_map(self.opendrive_map, self.config)
        start_index = grid.world_to_grid(start)
        goal_index = grid.nearest_free(grid.world_to_grid(goal))
        grid.require_free(start_index, "start")
        grid.require_free(goal_index, "goal")

        grid_path = _astar(grid, start_index, goal_index, self.config.right_lane_weight)
        return [grid.grid_to_world(index) for index in grid_path]

    def plan_to_object(self, start: Point, object_name: str) -> list[Point]:
        return self.plan_route_to_object(start, object_name).points

    def plan_route_to_object(self, start: Point, object_name: str) -> PlannedRoute:
        target = self.opendrive_map.find_object(object_name)
        if target.object_type != "parkingSpace":
            return PlannedRoute(
                target=target,
                target_yaw=None,
                segments=(PathSegment("forward", tuple(self.plan(start, target.center))),),
            )

        target_yaw = _head_out_yaw(target, self.opendrive_map.road_polygons)
        staging = _reverse_staging_point(target, target_yaw, self.config.reverse_staging_distance)
        forward_points = self.plan(start, staging)
        reverse_points = _line_points(forward_points[-1], target.center, self.config.resolution)
        return PlannedRoute(
            target=target,
            target_yaw=target_yaw,
            segments=(
                PathSegment("forward", tuple(forward_points)),
                PathSegment("reverse", tuple(reverse_points)),
            ),
        )


@dataclass(frozen=True)
class OccupancyGrid:
    min_x: float
    min_y: float
    width: int
    height: int
    resolution: float
    occupied: frozenset[GridIndex]
    lane_costs: dict[GridIndex, float]

    @classmethod
    def from_map(cls, opendrive_map: OpenDriveMap, config: GridPlannerConfig) -> "OccupancyGrid":
        min_x, max_x, min_y, max_y = opendrive_map.bounds(margin=2.0)
        width = int(math.ceil((max_x - min_x) / config.resolution)) + 1
        height = int(math.ceil((max_y - min_y) / config.resolution)) + 1
        occupied: set[GridIndex] = set()
        lane_costs: dict[GridIndex, float] = {}

        for ix in range(width):
            for iy in range(height):
                point = (min_x + ix * config.resolution, min_y + iy * config.resolution)
                if not _point_in_any_polygon(point, opendrive_map.road_polygons):
                    occupied.add((ix, iy))
                    continue
                if _point_blocked_by_objects(point, opendrive_map.objects, config.obstacle_padding):
                    occupied.add((ix, iy))
                    continue
                lane_costs[(ix, iy)] = _right_lane_cost(point, opendrive_map.road_guides)

        return cls(min_x, min_y, width, height, config.resolution, frozenset(occupied), lane_costs)

    def world_to_grid(self, point: Point) -> GridIndex:
        return (
            int(round((point[0] - self.min_x) / self.resolution)),
            int(round((point[1] - self.min_y) / self.resolution)),
        )

    def grid_to_world(self, index: GridIndex) -> Point:
        return (
            self.min_x + index[0] * self.resolution,
            self.min_y + index[1] * self.resolution,
        )

    def is_inside(self, index: GridIndex) -> bool:
        return 0 <= index[0] < self.width and 0 <= index[1] < self.height

    def is_free(self, index: GridIndex) -> bool:
        return self.is_inside(index) and index not in self.occupied

    def require_free(self, index: GridIndex, label: str) -> None:
        if not self.is_inside(index):
            raise ValueError(f"{label} is outside the planning grid")
        if index in self.occupied:
            raise ValueError(f"{label} is occupied")

    def nearest_free(self, index: GridIndex, max_radius: int = 20) -> GridIndex:
        if self.is_free(index):
            return index
        best: tuple[float, GridIndex] | None = None
        for radius in range(1, max_radius + 1):
            for dx in range(-radius, radius + 1):
                for dy in (-radius, radius):
                    candidate = (index[0] + dx, index[1] + dy)
                    if self.is_free(candidate):
                        distance = math.hypot(dx, dy)
                        if best is None or distance < best[0]:
                            best = (distance, candidate)
            for dy in range(-radius + 1, radius):
                for dx in (-radius, radius):
                    candidate = (index[0] + dx, index[1] + dy)
                    if self.is_free(candidate):
                        distance = math.hypot(dx, dy)
                        if best is None or distance < best[0]:
                            best = (distance, candidate)
            if best is not None:
                return best[1]
        raise ValueError("no free cell found near index")

    def neighbors(self, index: GridIndex, right_lane_weight: float) -> list[tuple[GridIndex, float]]:
        neighbors: list[tuple[GridIndex, float]] = []
        for dx, dy in (
            (-1, 0),
            (1, 0),
            (0, -1),
            (0, 1),
            (-1, -1),
            (-1, 1),
            (1, -1),
            (1, 1),
        ):
            next_index = (index[0] + dx, index[1] + dy)
            if self.is_free(next_index):
                base_cost = math.hypot(dx, dy)
                lane_cost = self.lane_costs.get(next_index, 0.0)
                neighbors.append((next_index, base_cost * (1.0 + right_lane_weight * lane_cost)))
        return neighbors


def load_opendrive_map(path: Path) -> OpenDriveMap:
    root = ET.parse(path).getroot()
    road_polygons: list[tuple[Point, ...]] = []
    road_guides: list[RoadGuide] = []
    objects: list[MapObject] = []

    for road in root.findall("road"):
        frame = _road_frame(road)
        road_polygons.append(_road_polygon(frame))
        road_guides.append(_right_lane_guide(frame))
        for obj in road.findall("./objects/object"):
            polygon = _object_polygon(frame, obj)
            if not polygon:
                continue
            center = frame.st_to_xy(_float_attr(obj, "s"), _float_attr(obj, "t"))
            objects.append(
                MapObject(
                    name=obj.get("name", ""),
                    object_type=obj.get("type", ""),
                    subtype=obj.get("subtype"),
                    polygon=tuple(polygon),
                    center=center,
                    heading=frame.heading + _float_attr(obj, "hdg"),
                    length=_float_attr(obj, "length"),
                    width=_float_attr(obj, "width"),
                    fill=obj.get("fill"),
                )
            )

    return OpenDriveMap(tuple(road_polygons), tuple(road_guides), tuple(objects))


def _float_attr(element: ET.Element, name: str, default: float = 0.0) -> float:
    value = element.get(name)
    return default if value is None else float(value)


def _road_frame(road: ET.Element) -> RoadFrame:
    geometry = road.find("./planView/geometry")
    if geometry is None or geometry.find("line") is None:
        raise ValueError("only straight line OpenDRIVE geometry is supported")

    left_width = sum(_float_attr(width, "a") for width in road.findall("./lanes/laneSection/left/lane/width"))
    right_width = sum(_float_attr(width, "a") for width in road.findall("./lanes/laneSection/right/lane/width"))
    return RoadFrame(
        x=_float_attr(geometry, "x"),
        y=_float_attr(geometry, "y"),
        heading=_float_attr(geometry, "hdg"),
        length=_float_attr(geometry, "length"),
        left_width=left_width,
        right_width=right_width,
    )


def _road_polygon(frame: RoadFrame) -> tuple[Point, ...]:
    return (
        frame.st_to_xy(0.0, -frame.right_width),
        frame.st_to_xy(frame.length, -frame.right_width),
        frame.st_to_xy(frame.length, frame.left_width),
        frame.st_to_xy(0.0, frame.left_width),
    )


def _right_lane_guide(frame: RoadFrame) -> RoadGuide:
    if frame.right_width <= 0.0:
        t = 0.0
        lane_width = max(frame.left_width, 1.0)
    else:
        t = -frame.right_width / 2.0
        lane_width = frame.right_width
    return RoadGuide(frame.st_to_xy(0.0, t), frame.st_to_xy(frame.length, t), frame.heading, lane_width)


def _object_polygon(frame: RoadFrame, obj: ET.Element) -> list[Point]:
    corners = obj.findall("./outline/cornerLocal")
    if not corners:
        return []
    obj_s = _float_attr(obj, "s")
    obj_t = _float_attr(obj, "t")
    obj_hdg = _float_attr(obj, "hdg")
    cos_h = math.cos(obj_hdg)
    sin_h = math.sin(obj_hdg)
    points: list[Point] = []
    for corner in corners:
        u = _float_attr(corner, "u")
        v = _float_attr(corner, "v")
        s = obj_s + u * cos_h - v * sin_h
        t = obj_t + u * sin_h + v * cos_h
        points.append(frame.st_to_xy(s, t))
    return points


def _point_blocked_by_objects(point: Point, objects: tuple[MapObject, ...], padding: float) -> bool:
    for obj in objects:
        if obj.object_type in {"pedestrianWalkway", "gate", "ramp"}:
            continue
        if _point_near_polygon(point, obj.polygon, padding):
            return True
    return False


def _point_in_any_polygon(point: Point, polygons: tuple[tuple[Point, ...], ...]) -> bool:
    return any(_point_in_polygon(point, polygon) for polygon in polygons)


def _point_near_polygon(point: Point, polygon: tuple[Point, ...], padding: float) -> bool:
    if _point_in_polygon(point, polygon):
        return True
    if padding <= 0.0:
        return False
    return any(
        _distance_to_segment(point, polygon[index], polygon[(index + 1) % len(polygon)]) <= padding
        for index in range(len(polygon))
    )


def _point_in_polygon(point: Point, polygon: tuple[Point, ...]) -> bool:
    x, y = point
    inside = False
    j = len(polygon) - 1
    for i, pi in enumerate(polygon):
        xi, yi = pi
        xj, yj = polygon[j]
        intersects = (yi > y) != (yj > y) and x < (xj - xi) * (y - yi) / (yj - yi) + xi
        if intersects:
            inside = not inside
        j = i
    return inside


def _distance_to_segment(point: Point, start: Point, end: Point) -> float:
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


def _right_lane_cost(point: Point, guides: tuple[RoadGuide, ...]) -> float:
    if not guides:
        return 0.0
    best_cost = math.inf
    for guide in guides:
        distance = _distance_to_segment(point, guide.start, guide.end)
        normalized = distance / max(guide.lane_width * 0.5, 0.1)
        best_cost = min(best_cost, min(normalized * normalized, 4.0))
    return 0.0 if best_cost is math.inf else best_cost


def _head_out_yaw(target: MapObject, road_polygons: tuple[tuple[Point, ...], ...]) -> float:
    axis = (math.cos(target.heading), math.sin(target.heading))
    half_length = target.length / 2.0
    candidates = (
        (target.center[0] + axis[0] * half_length, target.center[1] + axis[1] * half_length),
        (target.center[0] - axis[0] * half_length, target.center[1] - axis[1] * half_length),
    )
    outward = min(candidates, key=lambda point: _distance_to_nearest_polygon(point, road_polygons))
    return math.atan2(outward[1] - target.center[1], outward[0] - target.center[0])


def _distance_to_nearest_polygon(point: Point, polygons: tuple[tuple[Point, ...], ...]) -> float:
    distances: list[float] = []
    for polygon in polygons:
        if _point_in_polygon(point, polygon):
            return 0.0
        distances.extend(
            _distance_to_segment(point, polygon[index], polygon[(index + 1) % len(polygon)])
            for index in range(len(polygon))
        )
    return min(distances) if distances else math.inf


def _reverse_staging_point(target: MapObject, target_yaw: float, distance: float) -> Point:
    return (
        target.center[0] + math.cos(target_yaw) * distance,
        target.center[1] + math.sin(target_yaw) * distance,
    )


def _line_points(start: Point, end: Point, resolution: float) -> list[Point]:
    distance = math.hypot(end[0] - start[0], end[1] - start[1])
    steps = max(1, int(math.ceil(distance / resolution)))
    return [
        (
            start[0] + (end[0] - start[0]) * step / steps,
            start[1] + (end[1] - start[1]) * step / steps,
        )
        for step in range(steps + 1)
    ]


def _astar(
    grid: OccupancyGrid,
    start: GridIndex,
    goal: GridIndex,
    right_lane_weight: float,
) -> list[GridIndex]:
    open_heap: list[tuple[float, GridIndex]] = []
    heappush(open_heap, (0.0, start))
    came_from: dict[GridIndex, GridIndex] = {}
    g_score: dict[GridIndex, float] = {start: 0.0}

    while open_heap:
        _, current = heappop(open_heap)
        if current == goal:
            return _reconstruct_path(came_from, current)

        for neighbor, move_cost in grid.neighbors(current, right_lane_weight):
            tentative = g_score[current] + move_cost
            if tentative >= g_score.get(neighbor, math.inf):
                continue
            came_from[neighbor] = current
            g_score[neighbor] = tentative
            heappush(open_heap, (tentative + _heuristic(neighbor, goal), neighbor))

    raise ValueError("no route found")


def _heuristic(a: GridIndex, b: GridIndex) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def _reconstruct_path(came_from: dict[GridIndex, GridIndex], current: GridIndex) -> list[GridIndex]:
    path = [current]
    while current in came_from:
        current = came_from[current]
        path.append(current)
    path.reverse()
    return path
