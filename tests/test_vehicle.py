import math
import unittest

from hpasim.vehicle import (
    KinematicBicycleModel,
    VehicleControl,
    VehicleParams,
    VehicleState,
    normalize_angle,
)


class KinematicBicycleModelTest(unittest.TestCase):
    def test_straight_motion(self) -> None:
        model = KinematicBicycleModel()
        state = VehicleState(v=2.0)
        next_state = model.step(state, VehicleControl(), dt=1.0)

        self.assertAlmostEqual(next_state.x, 2.0)
        self.assertAlmostEqual(next_state.y, 0.0)
        self.assertAlmostEqual(next_state.yaw, 0.0)
        self.assertAlmostEqual(next_state.v, 2.0)

    def test_acceleration_updates_speed(self) -> None:
        model = KinematicBicycleModel()
        state = VehicleState(v=1.0)
        next_state = model.step(state, VehicleControl(acceleration=2.0), dt=0.5)

        self.assertAlmostEqual(next_state.v, 2.0)

    def test_front_steer_changes_yaw(self) -> None:
        model = KinematicBicycleModel(VehicleParams(wheelbase=2.0))
        state = VehicleState(v=2.0)
        next_state = model.step(state, VehicleControl(steer=math.atan(0.5)), dt=1.0)

        self.assertAlmostEqual(next_state.yaw, 0.5)

    def test_limits_control_and_speed(self) -> None:
        model = KinematicBicycleModel(VehicleParams(max_speed=3.0, max_acceleration=1.0))
        state = VehicleState(v=2.5)
        next_state = model.step(state, VehicleControl(steer=10.0, acceleration=10.0), dt=1.0)

        self.assertAlmostEqual(next_state.v, 3.0)
        self.assertAlmostEqual(
            abs(next_state.yaw),
            math.tan(model.params.max_steer) * state.v / model.params.wheelbase,
        )

    def test_normalize_angle(self) -> None:
        self.assertAlmostEqual(normalize_angle(math.pi), -math.pi)
        self.assertAlmostEqual(normalize_angle(3.0 * math.pi), -math.pi)
        self.assertAlmostEqual(normalize_angle(-3.0 * math.pi), -math.pi)

    def test_rejects_non_positive_dt(self) -> None:
        model = KinematicBicycleModel()
        with self.assertRaises(ValueError):
            model.step(VehicleState(), VehicleControl(), dt=0.0)


if __name__ == "__main__":
    unittest.main()
