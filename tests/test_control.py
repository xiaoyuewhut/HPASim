import unittest

from hpasim.control import LQRConfig, LQRController
from hpasim.trajectory import Trajectory, TrajectoryPoint
from hpasim.vehicle import KinematicBicycleModel, VehicleState


class LQRControllerTest(unittest.TestCase):
    def test_lqr_steers_toward_reference_line(self) -> None:
        trajectory = Trajectory(
            tuple(TrajectoryPoint(x=float(index), y=0.0, yaw=0.0, v=2.0, gear="forward", t=index * 0.25, s=index * 0.5) for index in range(20))
        )
        controller = LQRController(config=LQRConfig(dt=0.1))

        control, index = controller.control(VehicleState(x=0.0, y=1.0, yaw=0.0, v=2.0), trajectory)

        self.assertGreaterEqual(index, 0)
        self.assertLess(control.steer, 0.0)

    def test_lqr_closed_loop_reduces_lateral_error(self) -> None:
        trajectory = Trajectory(
            tuple(TrajectoryPoint(x=float(index), y=0.0, yaw=0.0, v=2.0, gear="forward", t=index * 0.5, s=index) for index in range(80))
        )
        controller = LQRController(config=LQRConfig(dt=0.1))
        model = KinematicBicycleModel()
        state = VehicleState(x=0.0, y=1.2, yaw=0.0, v=2.0)
        reference_index = 0

        for _ in range(30):
            control, reference_index = controller.control(state, trajectory, reference_index)
            state = model.step(state, control, 0.1)

        self.assertLess(abs(state.y), 1.2)


if __name__ == "__main__":
    unittest.main()
