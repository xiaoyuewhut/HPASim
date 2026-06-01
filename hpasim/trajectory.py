"""Reference trajectory generation from planned parking routes."""

from __future__ import annotations

from dataclasses import dataclass
import math

from hpasim.planner import PathSegment, PlannedRoute, Point
from hpasim.vehicle import normalize_angle


@dataclass(frozen=True)
class TrajectoryPoint:
    x: float
    y: float
    yaw: float
    v: float
    gear: str
    t: float
    s: float
    curvature: float = 0.0
    steer: float = 0.0
    acceleration: float = 0.0


@dataclass(frozen=True)
class Trajectory:
    points: tuple[TrajectoryPoint, ...]

    def __len__(self) -> int:
        return len(self.points)

    def __getitem__(self, index: int) -> TrajectoryPoint:
        return self.points[index]


@dataclass(frozen=True)
class TrajectoryPlannerConfig:
    spacing: float = 0.5
    forward_speed: float = 2.0
    reverse_speed: float = -1.0
    wheelbase: float = 2.8
    smoothing_iterations: int = 5
    max_acceleration: float = 1.2
    max_deceleration: float = 1.8
    lateral_acceleration_limit: float = 1.0
    curve_speed_floor: float = 0.45
    reverse_curve_speed_floor: float = 0.25
    parking_creep_distance: float = 2.0
    parking_creep_speed: float = 0.35
    reverse_tangent_scale: float = 1.2

    def __post_init__(self) -> None:
        if self.spacing <= 0.0:
            raise ValueError("spacing must be positive")
        if self.forward_speed <= 0.0:
            raise ValueError("forward_speed must be positive")
        if self.reverse_speed >= 0.0:
            raise ValueError("reverse_speed must be negative")
        if self.wheelbase <= 0.0:
            raise ValueError("wheelbase must be positive")
        if self.smoothing_iterations < 0:
            raise ValueError("smoothing_iterations must be non-negative")
        if self.max_acceleration <= 0.0:
            raise ValueError("max_acceleration must be positive")
        if self.max_deceleration <= 0.0:
            raise ValueError("max_deceleration must be positive")
        if self.lateral_acceleration_limit <= 0.0:
            raise ValueError("lateral_acceleration_limit must be positive")
        if self.curve_speed_floor <= 0.0:
            raise ValueError("curve_speed_floor must be positive")
        if self.reverse_curve_speed_floor <= 0.0:
            raise ValueError("reverse_curve_speed_floor must be positive")
        if self.parking_creep_distance <= 0.0:
            raise ValueError("parking_creep_distance must be positive")
        if self.parking_creep_speed <= 0.0:
            raise ValueError("parking_creep_speed must be positive")
        if self.reverse_tangent_scale <= 0.0:
            raise ValueError("reverse_tangent_scale must be positive")


@dataclass(frozen=True)
class TrajectoryPlanner:
    config: TrajectoryPlannerConfig = TrajectoryPlannerConfig()

    def plan(self, route: PlannedRoute) -> Trajectory:
        samples: list[tuple[Point, str]] = []
        segments = _prepare_segments(route, self.config)
        for segment in segments:
            segment_samples = _resample_segment(segment, self.config.spacing)
            if samples and segment_samples and samples[-1][0] == segment_samples[0][0]:
                samples.extend(segment_samples[1:])
            else:
                samples.extend(segment_samples)

        if len(samples) < 2:
            raise ValueError("route must contain at least two trajectory samples")

        distances = _cumulative_distances(samples)
        geometry_points: list[TrajectoryPoint] = []
        for index, (point, gear) in enumerate(samples):
            yaw = _sample_yaw(samples, index, route.target_yaw)
            geometry_points.append(TrajectoryPoint(point[0], point[1], yaw, 0.0, gear, 0.0, distances[index]))

        shaped_points = _with_curvature_and_steer(geometry_points, self.config.wheelbase)
        speeds = _speed_profile(shaped_points, distances, self.config)
        timed_points = _with_speed_and_time(shaped_points, speeds)
        return Trajectory(tuple(_with_acceleration(timed_points)))


def _prepare_segments(route: PlannedRoute, config: TrajectoryPlannerConfig) -> tuple[PathSegment, ...]:
    segments: list[PathSegment] = []
    for segment in route.segments:
        if segment.gear == "forward":
            segments.append(_smooth_segment(segment, config.smoothing_iterations))
            continue

        if (
            segment.gear == "reverse"
            and segments
            and segments[-1].gear == "forward"
            and route.target_yaw is not None
        ):
            start_vehicle_yaw = _polyline_end_yaw(segments[-1].points)
            segments.append(_curve_reverse_segment(segment, start_vehicle_yaw, route.target_yaw, config))
            continue

        segments.append(segment)
    return tuple(segments)


def _resample_segment(segment: PathSegment, spacing: float) -> list[tuple[Point, str]]:
    if len(segment.points) < 2:
        return [(point, segment.gear) for point in segment.points]

    samples: list[tuple[Point, str]] = [(segment.points[0], segment.gear)]
    remainder = spacing
    for start, end in zip(segment.points, segment.points[1:]):
        sx, sy = start
        ex, ey = end
        length = math.hypot(ex - sx, ey - sy)
        if length == 0.0:
            continue
        distance = remainder
        while distance <= length:
            ratio = distance / length
            samples.append(((
                sx + (ex - sx) * ratio,
                sy + (ey - sy) * ratio,
            ), segment.gear))
            distance += spacing
        remainder = distance - length

    if samples[-1][0] != segment.points[-1]:
        samples.append((segment.points[-1], segment.gear))
    return samples


def _smooth_segment(segment: PathSegment, iterations: int) -> PathSegment:
    if segment.gear == "reverse" or len(segment.points) < 3:
        return segment
    points = _simplify_polyline(segment.points)
    for _ in range(iterations):
        next_points = [points[0]]
        for start, end in zip(points, points[1:]):
            next_points.append((
                start[0] * 0.75 + end[0] * 0.25,
                start[1] * 0.75 + end[1] * 0.25,
            ))
            next_points.append((
                start[0] * 0.25 + end[0] * 0.75,
                start[1] * 0.25 + end[1] * 0.75,
            ))
        next_points.append(points[-1])
        points = next_points
    return PathSegment(segment.gear, tuple(points))


def _simplify_polyline(points: tuple[Point, ...], angle_tolerance: float = math.radians(5.0)) -> list[Point]:
    if len(points) <= 2:
        return list(points)
    simplified = [points[0]]
    prev_direction: tuple[float, float] | None = None
    for start, end in zip(points, points[1:]):
        dx = end[0] - start[0]
        dy = end[1] - start[1]
        length = math.hypot(dx, dy)
        if length == 0.0:
            continue
        direction = (dx / length, dy / length)
        if prev_direction is None:
            prev_direction = direction
            continue
        dot = max(-1.0, min(1.0, prev_direction[0] * direction[0] + prev_direction[1] * direction[1]))
        if math.acos(dot) > angle_tolerance:
            simplified.append(start)
        prev_direction = direction
    simplified.append(points[-1])
    return simplified


def _curve_reverse_segment(
    segment: PathSegment,
    start_vehicle_yaw: float,
    target_vehicle_yaw: float,
    config: TrajectoryPlannerConfig,
) -> PathSegment:
    if len(segment.points) < 2:
        return segment
    start = segment.points[0]
    end = segment.points[-1]
    distance = math.hypot(end[0] - start[0], end[1] - start[1])
    if distance == 0.0:
        return segment

    # While reversing, path motion is opposite to vehicle heading.
    start_motion_yaw = normalize_angle(start_vehicle_yaw + math.pi)
    end_motion_yaw = normalize_angle(target_vehicle_yaw + math.pi)
    curve = _hermite_curve(start, start_motion_yaw, end, end_motion_yaw, distance, config.reverse_tangent_scale)
    return PathSegment(segment.gear, tuple(curve))


def _polyline_end_yaw(points: tuple[Point, ...]) -> float:
    if len(points) < 2:
        return 0.0
    start = points[-2]
    end = points[-1]
    return math.atan2(end[1] - start[1], end[0] - start[0])


def _hermite_curve(
    start: Point,
    start_yaw: float,
    end: Point,
    end_yaw: float,
    distance: float,
    tangent_scale: float,
) -> list[Point]:
    tangent_length = max(distance * tangent_scale, 1.0)
    start_tangent = (math.cos(start_yaw) * tangent_length, math.sin(start_yaw) * tangent_length)
    end_tangent = (math.cos(end_yaw) * tangent_length, math.sin(end_yaw) * tangent_length)
    steps = max(12, int(math.ceil(distance / 0.5)))
    points: list[Point] = []
    for step in range(steps + 1):
        u = step / steps
        h00 = 2.0 * u**3 - 3.0 * u**2 + 1.0
        h10 = u**3 - 2.0 * u**2 + u
        h01 = -2.0 * u**3 + 3.0 * u**2
        h11 = u**3 - u**2
        points.append(
            (
                h00 * start[0] + h10 * start_tangent[0] + h01 * end[0] + h11 * end_tangent[0],
                h00 * start[1] + h10 * start_tangent[1] + h01 * end[1] + h11 * end_tangent[1],
            )
        )
    return points


def _cumulative_distances(samples: list[tuple[Point, str]]) -> list[float]:
    distances = [0.0]
    for (start, _), (end, _) in zip(samples, samples[1:]):
        distances.append(distances[-1] + math.hypot(end[0] - start[0], end[1] - start[1]))
    return distances


def _sample_yaw(samples: list[tuple[Point, str]], index: int, target_yaw: float | None) -> float:
    point, gear = samples[index]
    next_index = _same_gear_neighbor(samples, index, 1)
    prev_index = _same_gear_neighbor(samples, index, -1)
    if next_index is not None:
        next_point = samples[next_index][0]
        motion_yaw = math.atan2(next_point[1] - point[1], next_point[0] - point[0])
    elif prev_index is not None:
        prev_point = samples[prev_index][0]
        motion_yaw = math.atan2(point[1] - prev_point[1], point[0] - prev_point[0])
    else:
        motion_yaw = target_yaw or 0.0

    if gear == "reverse":
        return normalize_angle(motion_yaw + math.pi)
    return normalize_angle(motion_yaw)


def _same_gear_neighbor(samples: list[tuple[Point, str]], index: int, step: int) -> int | None:
    gear = samples[index][1]
    candidate = index + step
    if 0 <= candidate < len(samples) and samples[candidate][1] == gear:
        return candidate
    return None


def _with_curvature_and_steer(points: list[TrajectoryPoint], wheelbase: float) -> list[TrajectoryPoint]:
    updated: list[TrajectoryPoint] = []
    for index, point in enumerate(points):
        curvature = _curvature_at(points, index)
        steer = math.atan(wheelbase * curvature)
        updated.append(
            TrajectoryPoint(
                x=point.x,
                y=point.y,
                yaw=point.yaw,
                v=point.v,
                gear=point.gear,
                t=point.t,
                s=point.s,
                curvature=curvature,
                steer=steer,
                acceleration=point.acceleration,
            )
        )
    return updated


def _speed_profile(
    points: list[TrajectoryPoint],
    distances: list[float],
    config: TrajectoryPlannerConfig,
) -> list[float]:
    limits = [_speed_limit_at(points, distances, index, config) for index in range(len(points))]
    speeds = limits[:]

    for index, point in enumerate(points):
        if _is_stop_point(points, index):
            speeds[index] = 0.0
        if point.gear == "reverse":
            speeds[index] = min(speeds[index], abs(config.reverse_speed))

    for index in range(1, len(points)):
        ds = max(distances[index] - distances[index - 1], 0.0)
        reachable = math.sqrt(max(0.0, speeds[index - 1] ** 2 + 2.0 * config.max_acceleration * ds))
        speeds[index] = min(speeds[index], reachable)

    for index in range(len(points) - 2, -1, -1):
        ds = max(distances[index + 1] - distances[index], 0.0)
        reachable = math.sqrt(max(0.0, speeds[index + 1] ** 2 + 2.0 * config.max_deceleration * ds))
        speeds[index] = min(speeds[index], reachable)

    return [-speed if point.gear == "reverse" else speed for point, speed in zip(points, speeds)]


def _speed_limit_at(
    points: list[TrajectoryPoint],
    distances: list[float],
    index: int,
    config: TrajectoryPlannerConfig,
) -> float:
    point = points[index]
    nominal = abs(config.reverse_speed) if point.gear == "reverse" else config.forward_speed
    floor = config.reverse_curve_speed_floor if point.gear == "reverse" else config.curve_speed_floor
    curvature = abs(point.curvature)
    if curvature > 1e-6:
        nominal = min(nominal, max(floor, math.sqrt(config.lateral_acceleration_limit / curvature)))

    if point.gear == "reverse":
        distance_to_end = _distance_to_gear_end(points, distances, index)
        if distance_to_end < config.parking_creep_distance:
            ratio = max(0.0, distance_to_end / config.parking_creep_distance)
            nominal = min(nominal, config.parking_creep_speed + (abs(config.reverse_speed) - config.parking_creep_speed) * ratio)
    return nominal


def _is_stop_point(points: list[TrajectoryPoint], index: int) -> bool:
    return (
        index == 0
        or index == len(points) - 1
        or points[index].gear != points[index - 1].gear
        or (index < len(points) - 1 and points[index].gear != points[index + 1].gear)
    )


def _distance_to_gear_end(points: list[TrajectoryPoint], distances: list[float], index: int) -> float:
    gear = points[index].gear
    end = index
    while end < len(points) - 1 and points[end + 1].gear == gear:
        end += 1
    return distances[end] - distances[index]


def _with_speed_and_time(points: list[TrajectoryPoint], speeds: list[float]) -> list[TrajectoryPoint]:
    updated: list[TrajectoryPoint] = []
    t = 0.0
    for index, (point, speed) in enumerate(zip(points, speeds)):
        if updated:
            previous = updated[-1]
            ds = math.hypot(point.x - previous.x, point.y - previous.y)
            avg_speed = (abs(previous.v) + abs(speed)) * 0.5
            if avg_speed > 1e-3:
                t += ds / avg_speed
        updated.append(
            TrajectoryPoint(
                x=point.x,
                y=point.y,
                yaw=point.yaw,
                v=speed,
                gear=point.gear,
                t=t,
                s=point.s,
                curvature=point.curvature,
                steer=point.steer,
                acceleration=point.acceleration,
            )
        )
    return updated


def _with_acceleration(points: list[TrajectoryPoint]) -> list[TrajectoryPoint]:
    updated: list[TrajectoryPoint] = []
    for index, point in enumerate(points):
        acceleration = 0.0
        if index < len(points) - 1:
            dt = points[index + 1].t - point.t
            if dt > 1e-6:
                acceleration = (points[index + 1].v - point.v) / dt
        updated.append(
            TrajectoryPoint(
                x=point.x,
                y=point.y,
                yaw=point.yaw,
                v=point.v,
                gear=point.gear,
                t=point.t,
                s=point.s,
                curvature=point.curvature,
                steer=point.steer,
                acceleration=acceleration,
            )
        )
    return updated


def _curvature_at(points: list[TrajectoryPoint], index: int) -> float:
    if index == 0 or index >= len(points) - 1:
        return 0.0
    prev_point = points[index - 1]
    point = points[index]
    next_point = points[index + 1]
    ds = math.hypot(next_point.x - prev_point.x, next_point.y - prev_point.y)
    if ds == 0.0:
        return 0.0
    return normalize_angle(next_point.yaw - prev_point.yaw) / ds
