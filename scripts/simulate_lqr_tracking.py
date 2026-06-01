"""Simulate LQR tracking on route-derived HPASim reference trajectories."""

from pathlib import Path
import math
import sys


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from hpasim.control import LQRConfig, LQRController
from hpasim.map_payload import DEFAULT_SCENARIO, load_default_map
from hpasim.planner import GridPlannerConfig, GridRoutePlanner
from hpasim.trajectory import Trajectory, TrajectoryPlanner, TrajectoryPlannerConfig
from hpasim.vehicle import KinematicBicycleModel, VehicleState


def main() -> None:
    opendrive_map = load_default_map()
    route = GridRoutePlanner(
        opendrive_map,
        GridPlannerConfig(resolution=1.0, obstacle_padding=0.2),
    ).plan_route_to_object((DEFAULT_SCENARIO.ego_start[0], DEFAULT_SCENARIO.ego_start[1]), DEFAULT_SCENARIO.target_parking_space)
    trajectory = TrajectoryPlanner(TrajectoryPlannerConfig(spacing=0.5, forward_speed=1.2)).plan(route)

    forward_trajectory = Trajectory(trajectory.points[:160])
    forward_result = _simulate(forward_trajectory)

    print(f"trajectory_points={len(trajectory)}")
    print(f"forward_segment_points={len(forward_trajectory)}")
    print(f"forward_tracking_error={forward_result[0]:.3f}")
    print(f"forward_final_state={_format_state(forward_result[1])}")
    first_reverse_index = next((index for index, point in enumerate(trajectory.points) if point.gear == "reverse"), None)
    if first_reverse_index is None:
        print("reverse_segment_points=0")
        print("note=route handoff heading is not reverse-capable, so planner uses forward parking instead of fake reverse.")
        return

    reverse_trajectory = Trajectory(trajectory.points[first_reverse_index:])
    reverse_result = _simulate(reverse_trajectory)
    print(f"reverse_segment_points={len(reverse_trajectory)}")
    print(f"reverse_tracking_error={reverse_result[0]:.3f}")
    print(f"reverse_final_state={_format_state(reverse_result[1])}")
    print("note=LQR demo tracks forward and reverse references separately; full parking still needs a Hybrid A* maneuver planner.")


def _simulate(trajectory: Trajectory) -> tuple[float, VehicleState, int]:
    dt = 0.1
    model = KinematicBicycleModel()
    controller = LQRController(config=LQRConfig(dt=dt))
    first = trajectory[0]
    state = VehicleState(
        x=first.x,
        y=first.y,
        yaw=first.yaw,
        v=first.v,
    )
    reference_index = 0

    for _ in range(700):
        control, reference_index = controller.control(state, trajectory, reference_index)
        state = model.step(state, control, dt)
        if reference_index >= len(trajectory) - 2:
            break

    target = trajectory[min(reference_index, len(trajectory) - 1)]
    position_error = math.hypot(state.x - target.x, state.y - target.y)
    return position_error, state, reference_index


def _format_state(state: VehicleState) -> str:
    return f"{state.x:.3f},{state.y:.3f},{state.yaw:.3f},{state.v:.3f}"


if __name__ == "__main__":
    main()
