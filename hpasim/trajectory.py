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
    smoothing_iterations: int = 3
    speed_ramp_distance: float = 4.0

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
        if self.speed_ramp_distance <= 0.0:
            raise ValueError("speed_ramp_distance must be positive")


@dataclass(frozen=True)
class TrajectoryPlanner:
    config: TrajectoryPlannerConfig = TrajectoryPlannerConfig()

    def plan(self, route: PlannedRoute) -> Trajectory:
        samples: list[tuple[Point, str]] = []
        for segment in route.segments:
            smoothed_segment = _smooth_segment(segment, self.config.smoothing_iterations)
            segment_samples = _resample_segment(smoothed_segment, self.config.spacing)
            if samples and segment_samples and samples[-1][0] == segment_samples[0][0]:
                samples.extend(segment_samples[1:])
            else:
                samples.extend(segment_samples)

        if len(samples) < 2:
            raise ValueError("route must contain at least two trajectory samples")

        distances = _cumulative_distances(samples)
        points: list[TrajectoryPoint] = []
        t = 0.0
        for index, (point, gear) in enumerate(samples):
            yaw = _sample_yaw(samples, index, route.target_yaw)
            v = _sample_speed(samples, distances, index, self.config)
            if points:
                ds = math.hypot(point[0] - points[-1].x, point[1] - points[-1].y)
                avg_speed = max((abs(points[-1].v) + abs(v)) * 0.5, 0.2)
                t += ds / avg_speed
            points.append(TrajectoryPoint(point[0], point[1], yaw, v, gear, t, distances[index]))

        return Trajectory(tuple(_with_feedforward(points, self.config.wheelbase)))


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
    points = list(segment.points)
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


def _cumulative_distances(samples: list[tuple[Point, str]]) -> list[float]:
    distances = [0.0]
    for (start, _), (end, _) in zip(samples, samples[1:]):
        distances.append(distances[-1] + math.hypot(end[0] - start[0], end[1] - start[1]))
    return distances


def _sample_speed(
    samples: list[tuple[Point, str]],
    distances: list[float],
    index: int,
    config: TrajectoryPlannerConfig,
) -> float:
    gear = samples[index][1]
    nominal = config.reverse_speed if gear == "reverse" else config.forward_speed
    signed_scale = min(
        1.0,
        _distance_since_gear_start(samples, distances, index) / config.speed_ramp_distance,
        _distance_to_gear_end(samples, distances, index) / config.speed_ramp_distance,
    )
    if index == 0 or index == len(samples) - 1:
        signed_scale = 0.0
    return nominal * signed_scale


def _distance_since_gear_start(samples: list[tuple[Point, str]], distances: list[float], index: int) -> float:
    gear = samples[index][1]
    start = index
    while start > 0 and samples[start - 1][1] == gear:
        start -= 1
    return distances[index] - distances[start]


def _distance_to_gear_end(samples: list[tuple[Point, str]], distances: list[float], index: int) -> float:
    gear = samples[index][1]
    end = index
    while end < len(samples) - 1 and samples[end + 1][1] == gear:
        end += 1
    return distances[end] - distances[index]


def _sample_yaw(samples: list[tuple[Point, str]], index: int, target_yaw: float | None) -> float:
    point, gear = samples[index]
    if index < len(samples) - 1:
        next_point = samples[index + 1][0]
        motion_yaw = math.atan2(next_point[1] - point[1], next_point[0] - point[0])
    else:
        prev_point = samples[index - 1][0]
        motion_yaw = math.atan2(point[1] - prev_point[1], point[0] - prev_point[0])

    if gear == "reverse":
        if index == len(samples) - 1 and target_yaw is not None:
            return normalize_angle(target_yaw)
        return normalize_angle(motion_yaw + math.pi)
    return normalize_angle(motion_yaw)


def _with_feedforward(points: list[TrajectoryPoint], wheelbase: float) -> list[TrajectoryPoint]:
    updated: list[TrajectoryPoint] = []
    for index, point in enumerate(points):
        curvature = _curvature_at(points, index)
        steer = math.atan(wheelbase * curvature)
        acceleration = 0.0
        if index < len(points) - 1:
            dt = max(points[index + 1].t - point.t, 1e-6)
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
                curvature=curvature,
                steer=steer,
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
