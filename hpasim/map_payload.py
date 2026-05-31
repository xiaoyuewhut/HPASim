"""JSON-ready map payloads for the interactive parking viewer."""

from __future__ import annotations

from pathlib import Path
from typing import Any

from hpasim.parking_scenarios import SCENARIOS, ScenarioSpec
from hpasim.planner import OpenDriveMap, load_opendrive_map


PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_SCENARIO = SCENARIOS[0]


def default_map_path(scenario: ScenarioSpec = DEFAULT_SCENARIO) -> Path:
    return PROJECT_ROOT / "maps" / "opendrive" / f"{scenario.key}.xodr"


def load_default_map() -> OpenDriveMap:
    return load_opendrive_map(default_map_path())


def build_map_payload(opendrive_map: OpenDriveMap, scenario: ScenarioSpec = DEFAULT_SCENARIO) -> dict[str, Any]:
    min_x, max_x, min_y, max_y = opendrive_map.bounds(margin=2.0)
    return {
        "scenario": {
            "key": scenario.key,
            "name": scenario.name,
            "targetParkingSpace": scenario.target_parking_space,
            "egoStart": {
                "x": scenario.ego_start[0],
                "y": scenario.ego_start[1],
                "yaw": scenario.ego_start[2],
            },
        },
        "bounds": {
            "minX": min_x,
            "maxX": max_x,
            "minY": min_y,
            "maxY": max_y,
        },
        "roads": [{"polygon": _points(polygon)} for polygon in opendrive_map.road_polygons],
        "objects": [
            {
                "name": obj.name,
                "type": obj.object_type,
                "subtype": obj.subtype,
                "center": {"x": obj.center[0], "y": obj.center[1]},
                "heading": obj.heading,
                "length": obj.length,
                "width": obj.width,
                "fill": obj.fill,
                "polygon": _points(obj.polygon),
            }
            for obj in opendrive_map.objects
        ],
    }


def _points(points: tuple[tuple[float, float], ...]) -> list[dict[str, float]]:
    return [{"x": x, "y": y} for x, y in points]
