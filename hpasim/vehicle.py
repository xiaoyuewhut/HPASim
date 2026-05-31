"""Kinematic front-steering vehicle model."""

from __future__ import annotations

from dataclasses import dataclass
import math


def normalize_angle(angle: float) -> float:
    """Wrap an angle to [-pi, pi)."""

    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def clamp(value: float, lower: float, upper: float) -> float:
    return min(max(value, lower), upper)


@dataclass(frozen=True)
class VehicleParams:
    """Physical and control limits for a front-steering vehicle."""

    wheelbase: float = 2.8
    max_steer: float = math.radians(35.0)
    max_speed: float = 15.0
    min_speed: float = -5.0
    max_acceleration: float = 3.0
    min_acceleration: float = -6.0

    def __post_init__(self) -> None:
        if self.wheelbase <= 0.0:
            raise ValueError("wheelbase must be positive")
        if self.max_steer <= 0.0 or self.max_steer >= math.pi / 2.0:
            raise ValueError("max_steer must be in (0, pi/2)")
        if self.min_speed > self.max_speed:
            raise ValueError("min_speed must be less than or equal to max_speed")
        if self.min_acceleration > self.max_acceleration:
            raise ValueError("min_acceleration must be less than or equal to max_acceleration")


@dataclass(frozen=True)
class VehicleState:
    """Vehicle state in a world frame."""

    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    v: float = 0.0

    def normalized(self) -> "VehicleState":
        return VehicleState(self.x, self.y, normalize_angle(self.yaw), self.v)


@dataclass(frozen=True)
class VehicleControl:
    """Control input for the kinematic bicycle model."""

    steer: float = 0.0
    acceleration: float = 0.0


@dataclass(frozen=True)
class KinematicBicycleModel:
    """Front-wheel-steering kinematic bicycle model.

    State:
        x, y: rear-axle reference position in meters.
        yaw: vehicle heading in radians.
        v: longitudinal velocity in meters per second.

    Control:
        steer: front wheel steering angle in radians.
        acceleration: longitudinal acceleration in meters per second squared.
    """

    params: VehicleParams = VehicleParams()

    def limit_control(self, control: VehicleControl) -> VehicleControl:
        return VehicleControl(
            steer=clamp(control.steer, -self.params.max_steer, self.params.max_steer),
            acceleration=clamp(
                control.acceleration,
                self.params.min_acceleration,
                self.params.max_acceleration,
            ),
        )

    def derivatives(self, state: VehicleState, control: VehicleControl) -> VehicleState:
        limited = self.limit_control(control)
        return VehicleState(
            x=state.v * math.cos(state.yaw),
            y=state.v * math.sin(state.yaw),
            yaw=state.v / self.params.wheelbase * math.tan(limited.steer),
            v=limited.acceleration,
        )

    def step(self, state: VehicleState, control: VehicleControl, dt: float) -> VehicleState:
        if dt <= 0.0:
            raise ValueError("dt must be positive")

        limited = self.limit_control(control)
        dx = state.v * math.cos(state.yaw)
        dy = state.v * math.sin(state.yaw)
        dyaw = state.v / self.params.wheelbase * math.tan(limited.steer)
        next_v = clamp(
            state.v + limited.acceleration * dt,
            self.params.min_speed,
            self.params.max_speed,
        )
        return VehicleState(
            x=state.x + dx * dt,
            y=state.y + dy * dt,
            yaw=normalize_angle(state.yaw + dyaw * dt),
            v=next_v,
        )

    def simulate(
        self,
        initial_state: VehicleState,
        controls: list[VehicleControl],
        dt: float,
    ) -> list[VehicleState]:
        states = [initial_state.normalized()]
        state = states[0]
        for control in controls:
            state = self.step(state, control, dt)
            states.append(state)
        return states
