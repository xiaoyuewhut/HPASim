"""Serve the interactive HPASim parking viewer."""

from __future__ import annotations

from http import HTTPStatus
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
import argparse
import json
from pathlib import Path
import sys
from typing import Any


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from hpasim.map_payload import DEFAULT_SCENARIO, build_map_payload, load_default_map
from hpasim.planner import GridPlannerConfig, GridRoutePlanner
from hpasim.trajectory import TrajectoryPlanner, TrajectoryPlannerConfig


VIEWER_DIR = PROJECT_ROOT / "viewer"


class ViewerState:
    def __init__(self) -> None:
        self.opendrive_map = load_default_map()
        self.map_payload = build_map_payload(self.opendrive_map)
        self.planner = GridRoutePlanner(
            self.opendrive_map,
            GridPlannerConfig(resolution=1.0, obstacle_padding=0.2),
        )
        self.trajectory_planner = TrajectoryPlanner(TrajectoryPlannerConfig(spacing=0.5, forward_speed=1.2))


class ViewerHandler(SimpleHTTPRequestHandler):
    state: ViewerState

    def __init__(self, *args: Any, **kwargs: Any) -> None:
        super().__init__(*args, directory=str(VIEWER_DIR), **kwargs)

    def do_GET(self) -> None:
        if self.path == "/":
            self.path = "/index.html"
        if self.path == "/api/map":
            self._write_json(HTTPStatus.OK, self.state.map_payload)
            return
        super().do_GET()

    def do_POST(self) -> None:
        if self.path == "/api/plan":
            self._handle_plan()
            return
        if self.path == "/api/drive":
            self._handle_drive()
            return
        self._write_json(HTTPStatus.NOT_FOUND, {"error": "unknown endpoint"})

    def _handle_plan(self) -> None:
        try:
            body = self._read_json()
            target = str(body["target"])
            start = body.get("start") or {}
            x = float(start.get("x", DEFAULT_SCENARIO.ego_start[0]))
            y = float(start.get("y", DEFAULT_SCENARIO.ego_start[1]))
            route = self.state.planner.plan_route_to_object((x, y), target)
        except KeyError as exc:
            self._write_json(HTTPStatus.BAD_REQUEST, {"error": f"missing or unknown field: {exc}"})
            return
        except ValueError as exc:
            self._write_json(HTTPStatus.BAD_REQUEST, {"error": str(exc)})
            return

        self._write_json(
            HTTPStatus.OK,
            {
                "target": target,
                "targetYaw": route.target_yaw,
                "path": [{"x": x, "y": y} for x, y in route.points],
                "segments": [
                    {
                        "gear": segment.gear,
                        "points": [{"x": x, "y": y} for x, y in segment.points],
                    }
                    for segment in route.segments
                ],
            },
        )

    def _handle_drive(self) -> None:
        try:
            body = self._read_json()
            target = str(body["target"])
            start = body.get("start") or {}
            x = float(start.get("x", DEFAULT_SCENARIO.ego_start[0]))
            y = float(start.get("y", DEFAULT_SCENARIO.ego_start[1]))
            route = self.state.planner.plan_route_to_object((x, y), target)
            trajectory = self.state.trajectory_planner.plan(route)
        except KeyError as exc:
            self._write_json(HTTPStatus.BAD_REQUEST, {"error": f"missing or unknown field: {exc}"})
            return
        except ValueError as exc:
            self._write_json(HTTPStatus.BAD_REQUEST, {"error": str(exc)})
            return

        self._write_json(
            HTTPStatus.OK,
            {
                "target": target,
                "points": [
                    {
                        "x": point.x,
                        "y": point.y,
                        "yaw": point.yaw,
                        "v": point.v,
                        "gear": point.gear,
                        "t": point.t,
                        "s": point.s,
                    }
                    for point in trajectory.points
                ],
            },
        )

    def _read_json(self) -> dict[str, Any]:
        length = int(self.headers.get("Content-Length", "0"))
        data = self.rfile.read(length)
        if not data:
            return {}
        return json.loads(data.decode("utf-8"))

    def _write_json(self, status: HTTPStatus, payload: dict[str, Any]) -> None:
        encoded = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(encoded)))
        self.end_headers()
        self.wfile.write(encoded)

    def log_message(self, format: str, *args: Any) -> None:
        sys.stderr.write(f"[viewer] {self.address_string()} {format % args}\n")

def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8000)
    args = parser.parse_args()

    ViewerHandler.state = ViewerState()
    server = ThreadingHTTPServer((args.host, args.port), ViewerHandler)
    print(f"Interactive parking viewer: http://{args.host}:{args.port}")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
