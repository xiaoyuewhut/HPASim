"""LQR trajectory tracking controller for the kinematic bicycle model."""

from __future__ import annotations

from dataclasses import dataclass, field
import math

import numpy as np

from hpasim.trajectory import Trajectory, TrajectoryPoint
from hpasim.vehicle import VehicleControl, VehicleParams, VehicleState, clamp, normalize_angle


@dataclass(frozen=True)
class LQRConfig:
    dt: float = 0.1
    q: tuple[float, float, float] = (3.0, 5.0, 1.0)
    r: tuple[float, float] = (2.0, 0.5)
    lookahead_points: int = 3
    advance_distance: float = 1.6
    max_iterations: int = 100
    tolerance: float = 1e-6

    def __post_init__(self) -> None:
        if self.dt <= 0.0:
            raise ValueError("dt must be positive")
        if len(self.q) != 3:
            raise ValueError("q must contain three weights")
        if len(self.r) != 2:
            raise ValueError("r must contain two weights")
        if any(weight <= 0.0 for weight in self.q + self.r):
            raise ValueError("LQR weights must be positive")
        if self.lookahead_points < 0:
            raise ValueError("lookahead_points must be non-negative")
        if self.advance_distance <= 0.0:
            raise ValueError("advance_distance must be positive")
        if self.max_iterations <= 0:
            raise ValueError("max_iterations must be positive")
        if self.tolerance <= 0.0:
            raise ValueError("tolerance must be positive")


@dataclass(frozen=True)
class LQRController:
    params: VehicleParams = VehicleParams()
    config: LQRConfig = LQRConfig()
    q_matrix: np.ndarray = field(init=False, repr=False)
    r_matrix: np.ndarray = field(init=False, repr=False)

    def __post_init__(self) -> None:
        object.__setattr__(self, "q_matrix", np.diag(self.config.q))
        object.__setattr__(self, "r_matrix", np.diag(self.config.r))

    def control(
        self,
        state: VehicleState,
        trajectory: Trajectory,
        previous_index: int = 0,
    ) -> tuple[VehicleControl, int]:
        reference_index = self.nearest_reference_index(state, trajectory, previous_index)
        reference = trajectory[reference_index]
        lateral_reference = trajectory[min(reference_index + self.config.lookahead_points, len(trajectory.points) - 1)]
        error = _tracking_error(state, lateral_reference, reference.v)
        a_matrix, b_matrix = _linearized_error_model(lateral_reference, self.params.wheelbase, self.config.dt)
        gain = _dlqr(a_matrix, b_matrix, self.q_matrix, self.r_matrix, self.config)
        correction = -gain @ error
        steer = clamp(lateral_reference.steer + float(correction[0]), -self.params.max_steer, self.params.max_steer)
        acceleration = clamp(
            reference.acceleration + float(correction[1]),
            self.params.min_acceleration,
            self.params.max_acceleration,
        )
        return VehicleControl(steer=steer, acceleration=acceleration), reference_index

    def nearest_reference_index(
        self,
        state: VehicleState,
        trajectory: Trajectory,
        previous_index: int = 0,
        search_window: int = 20,
    ) -> int:
        if not trajectory.points:
            raise ValueError("trajectory is empty")
        start = max(0, previous_index)
        end = min(len(trajectory.points), start + search_window)
        if start >= len(trajectory.points):
            return len(trajectory.points) - 1
        while start < len(trajectory.points) - 1:
            point = trajectory[start]
            if math.hypot(state.x - point.x, state.y - point.y) > self.config.advance_distance:
                break
            start += 1
        best_index = start
        best_distance = math.inf
        for index in range(start, end):
            point = trajectory[index]
            distance = math.hypot(state.x - point.x, state.y - point.y)
            if distance < best_distance:
                best_distance = distance
                best_index = index
        return best_index


def _tracking_error(state: VehicleState, reference: TrajectoryPoint, reference_speed: float) -> np.ndarray:
    dx = state.x - reference.x
    dy = state.y - reference.y
    sin_yaw = math.sin(reference.yaw)
    cos_yaw = math.cos(reference.yaw)
    lateral_error = -sin_yaw * dx + cos_yaw * dy
    yaw_error = normalize_angle(state.yaw - reference.yaw)
    speed_error = state.v - reference_speed
    return np.array([lateral_error, yaw_error, speed_error], dtype=float)


def _linearized_error_model(reference: TrajectoryPoint, wheelbase: float, dt: float) -> tuple[np.ndarray, np.ndarray]:
    v = reference.v
    steer = reference.steer
    a_matrix = np.eye(3)
    a_matrix[0, 1] = v * dt

    b_matrix = np.zeros((3, 2))
    b_matrix[1, 0] = v / (wheelbase * max(math.cos(steer) ** 2, 1e-3)) * dt
    b_matrix[2, 1] = dt
    return a_matrix, b_matrix


def _dlqr(
    a_matrix: np.ndarray,
    b_matrix: np.ndarray,
    q_matrix: np.ndarray,
    r_matrix: np.ndarray,
    config: LQRConfig,
) -> np.ndarray:
    p_matrix = q_matrix.copy()
    for _ in range(config.max_iterations):
        bt_p = b_matrix.T @ p_matrix
        next_p = (
            a_matrix.T @ p_matrix @ a_matrix
            - a_matrix.T
            @ p_matrix
            @ b_matrix
            @ np.linalg.solve(r_matrix + bt_p @ b_matrix, bt_p @ a_matrix)
            + q_matrix
        )
        if np.max(np.abs(next_p - p_matrix)) < config.tolerance:
            p_matrix = next_p
            break
        p_matrix = next_p
    return np.linalg.solve(r_matrix + b_matrix.T @ p_matrix @ b_matrix, b_matrix.T @ p_matrix @ a_matrix)
