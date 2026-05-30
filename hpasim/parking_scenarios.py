"""Definitions and OpenDRIVE generation for memory parking test maps."""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Iterable
import math
import xml.etree.ElementTree as ET


PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_MAP_DIR = PROJECT_ROOT / "maps" / "opendrive"


@dataclass(frozen=True)
class RoadSpec:
    """Straight OpenDRIVE road used as the local reference line."""

    road_id: int
    name: str
    length: float
    x: float
    y: float
    heading: float = 0.0
    lane_width: float = 3.5
    left_lanes: int = 1
    right_lanes: int = 1


@dataclass(frozen=True)
class BoxObject:
    """A rectangular map object expressed in the road's local s/t frame."""

    object_id: int
    name: str
    object_type: str
    s: float
    t: float
    length: float
    width: float
    heading: float = 0.0
    subtype: str | None = None
    fill: str | None = None


@dataclass(frozen=True)
class ScenarioSpec:
    """Complete parking scenario specification."""

    key: str
    name: str
    description: str
    ego_start: tuple[float, float, float]
    target_parking_space: str
    roads: tuple[RoadSpec, ...]
    objects: tuple[BoxObject, ...] = field(default_factory=tuple)


def _box_corners(length: float, width: float, heading: float) -> list[tuple[float, float]]:
    half_l = length / 2.0
    half_w = width / 2.0
    local = [(-half_l, -half_w), (half_l, -half_w), (half_l, half_w), (-half_l, half_w)]
    cos_h = math.cos(heading)
    sin_h = math.sin(heading)
    return [
        (u * cos_h - v * sin_h, u * sin_h + v * cos_h)
        for u, v in local
    ]


def _add_lane(parent: ET.Element, lane_id: int, lane_type: str, width: float) -> None:
    lane = ET.SubElement(parent, "lane", id=str(lane_id), type=lane_type, level="false")
    ET.SubElement(lane, "link")
    ET.SubElement(
        lane,
        "width",
        sOffset="0.0",
        a=f"{width:.6f}",
        b="0.0",
        c="0.0",
        d="0.0",
    )
    ET.SubElement(lane, "roadMark", sOffset="0.0", type="broken", weight="standard", color="standard", width="0.12")


def _add_road(root: ET.Element, road: RoadSpec, objects: Iterable[BoxObject]) -> None:
    road_el = ET.SubElement(
        root,
        "road",
        name=road.name,
        length=f"{road.length:.6f}",
        id=str(road.road_id),
        junction="-1",
    )
    ET.SubElement(road_el, "link")
    ET.SubElement(road_el, "type", s="0.0", type="town")
    plan_view = ET.SubElement(road_el, "planView")
    geometry = ET.SubElement(
        plan_view,
        "geometry",
        s="0.0",
        x=f"{road.x:.6f}",
        y=f"{road.y:.6f}",
        hdg=f"{road.heading:.6f}",
        length=f"{road.length:.6f}",
    )
    ET.SubElement(geometry, "line")
    ET.SubElement(road_el, "elevationProfile")
    ET.SubElement(road_el, "lateralProfile")

    lanes = ET.SubElement(road_el, "lanes")
    lane_section = ET.SubElement(lanes, "laneSection", s="0.0")
    left = ET.SubElement(lane_section, "left")
    for lane_id in range(road.left_lanes, 0, -1):
        _add_lane(left, lane_id, "driving", road.lane_width)
    center = ET.SubElement(lane_section, "center")
    ET.SubElement(center, "lane", id="0", type="none", level="false")
    right = ET.SubElement(lane_section, "right")
    for lane_id in range(-1, -road.right_lanes - 1, -1):
        _add_lane(right, lane_id, "driving", road.lane_width)

    objects_el = ET.SubElement(road_el, "objects")
    for obj in objects:
        obj_el = ET.SubElement(
            objects_el,
            "object",
            id=str(obj.object_id),
            name=obj.name,
            type=obj.object_type,
            s=f"{obj.s:.6f}",
            t=f"{obj.t:.6f}",
            zOffset="0.0",
            validLength="0.0",
            orientation="none",
            length=f"{obj.length:.6f}",
            width=f"{obj.width:.6f}",
            hdg=f"{obj.heading:.6f}",
        )
        if obj.subtype:
            obj_el.set("subtype", obj.subtype)
        if obj.fill:
            obj_el.set("fill", obj.fill)
        if obj.object_type == "parkingSpace":
            ET.SubElement(obj_el, "parkingSpace", access="all")
        outline = ET.SubElement(obj_el, "outline")
        for index, (u, v) in enumerate(_box_corners(obj.length, obj.width, obj.heading)):
            ET.SubElement(
                outline,
                "cornerLocal",
                id=str(index),
                u=f"{u:.6f}",
                v=f"{v:.6f}",
                z="0.0",
                height="0.0",
            )


def _indent(element: ET.Element, level: int = 0) -> None:
    pad = "\n" + level * "  "
    if len(element):
        if not element.text or not element.text.strip():
            element.text = pad + "  "
        for child in element:
            _indent(child, level + 1)
        if not child.tail or not child.tail.strip():
            child.tail = pad
    if level and (not element.tail or not element.tail.strip()):
        element.tail = pad


def build_opendrive_xml(scenario: ScenarioSpec) -> ET.ElementTree:
    root = ET.Element("OpenDRIVE")
    ET.SubElement(
        root,
        "header",
        revMajor="1",
        revMinor="6",
        name=scenario.name,
        version="1.00",
        date="2026-05-30T00:00:00",
        north="120.0",
        south="-40.0",
        east="120.0",
        west="-40.0",
        vendor="HPASim",
    )
    for road in scenario.roads:
        _add_road(root, road, scenario.objects)
    _indent(root)
    return ET.ElementTree(root)


def write_scenario(scenario: ScenarioSpec, output_dir: Path = DEFAULT_MAP_DIR) -> Path:
    output_dir.mkdir(parents=True, exist_ok=True)
    path = output_dir / f"{scenario.key}.xodr"
    build_opendrive_xml(scenario).write(path, encoding="utf-8", xml_declaration=True)
    return path


def write_all_scenarios(output_dir: Path = DEFAULT_MAP_DIR) -> list[Path]:
    return [write_scenario(scenario, output_dir) for scenario in SCENARIOS]


SCENARIOS: tuple[ScenarioSpec, ...] = (
    ScenarioSpec(
        key="parallel_curb_slot",
        name="Parallel curb parking slot",
        description="Urban curbside parallel parking with parked vehicles before and after the target slot.",
        ego_start=(6.0, -1.75, 0.0),
        target_parking_space="target_parallel_01",
        roads=(RoadSpec(road_id=1, name="two_lane_urban_street", length=58.0, x=0.0, y=0.0),),
        objects=(
            BoxObject(101, "target_parallel_01", "parkingSpace", 30.0, -5.55, 6.2, 2.4, subtype="target", fill="#57c7ff"),
            BoxObject(102, "front_parked_vehicle", "vehicle", 37.0, -5.55, 4.7, 2.0, fill="#8f8f8f"),
            BoxObject(103, "rear_parked_vehicle", "vehicle", 22.8, -5.55, 4.7, 2.0, fill="#8f8f8f"),
            BoxObject(104, "right_curb", "curbstone", 29.0, -7.05, 48.0, 0.25, fill="#444444"),
        ),
    ),
    ScenarioSpec(
        key="perpendicular_bay_row",
        name="Perpendicular parking bay row",
        description="Parking-lot aisle with perpendicular bays on both sides and one occupied neighbor.",
        ego_start=(8.0, 0.0, 0.0),
        target_parking_space="target_perpendicular_01",
        roads=(RoadSpec(road_id=1, name="parking_lot_aisle", length=52.0, x=0.0, y=0.0),),
        objects=(
            BoxObject(201, "target_perpendicular_01", "parkingSpace", 28.0, 6.85, 5.2, 2.6, math.pi / 2, subtype="target", fill="#57c7ff"),
            BoxObject(202, "left_neighbor_space", "parkingSpace", 24.9, 6.85, 5.2, 2.6, math.pi / 2, fill="#e6e6e6"),
            BoxObject(203, "right_neighbor_space", "parkingSpace", 31.1, 6.85, 5.2, 2.6, math.pi / 2, fill="#e6e6e6"),
            BoxObject(204, "occupied_neighbor_vehicle", "vehicle", 31.1, 6.85, 4.6, 2.0, math.pi / 2, fill="#8f8f8f"),
            BoxObject(205, "opposite_row_space", "parkingSpace", 27.0, -6.85, 5.2, 2.6, -math.pi / 2, fill="#e6e6e6"),
            BoxObject(206, "wheel_stop", "barrier", 28.0, 9.55, 2.1, 0.18, fill="#555555"),
        ),
    ),
    ScenarioSpec(
        key="angled_60deg_bay",
        name="Sixty degree angled bay",
        description="Forward-search parking in an angled bay layout with a constrained exit side.",
        ego_start=(7.0, -0.8, 0.0),
        target_parking_space="target_angled_01",
        roads=(RoadSpec(road_id=1, name="angled_parking_aisle", length=56.0, x=0.0, y=0.0),),
        objects=(
            BoxObject(301, "target_angled_01", "parkingSpace", 31.0, 6.4, 5.3, 2.6, math.radians(60.0), subtype="target", fill="#57c7ff"),
            BoxObject(302, "angled_left_space", "parkingSpace", 27.9, 5.9, 5.3, 2.6, math.radians(60.0), fill="#e6e6e6"),
            BoxObject(303, "angled_right_space", "parkingSpace", 34.1, 6.9, 5.3, 2.6, math.radians(60.0), fill="#e6e6e6"),
            BoxObject(304, "angled_occupied_vehicle", "vehicle", 27.9, 5.9, 4.5, 2.0, math.radians(60.0), fill="#8f8f8f"),
            BoxObject(305, "exit_side_barrier", "barrier", 40.0, 4.0, 8.0, 0.25, math.radians(60.0), fill="#555555"),
        ),
    ),
    ScenarioSpec(
        key="narrow_garage_slot",
        name="Narrow garage slot",
        description="Memory parking into a tight private garage-like slot bounded by walls.",
        ego_start=(5.0, 0.0, 0.0),
        target_parking_space="target_garage_01",
        roads=(RoadSpec(road_id=1, name="narrow_private_drive", length=42.0, x=0.0, y=0.0, lane_width=3.0),),
        objects=(
            BoxObject(401, "target_garage_01", "parkingSpace", 30.5, 5.65, 5.8, 2.8, math.pi / 2, subtype="target", fill="#57c7ff"),
            BoxObject(402, "left_wall", "barrier", 27.4, 7.25, 6.2, 0.25, math.pi / 2, fill="#555555"),
            BoxObject(403, "right_wall", "barrier", 33.6, 7.25, 6.2, 0.25, math.pi / 2, fill="#555555"),
            BoxObject(404, "back_wall", "barrier", 30.5, 8.85, 3.6, 0.25, fill="#555555"),
            BoxObject(405, "driveway_edge", "curbstone", 24.0, -3.15, 34.0, 0.2, fill="#444444"),
        ),
    ),
)


def scenario_table() -> str:
    lines = ["| Scenario | Target | Purpose |", "| --- | --- | --- |"]
    for scenario in SCENARIOS:
        lines.append(
            f"| `{scenario.key}` | `{scenario.target_parking_space}` | {scenario.description} |"
        )
    return "\n".join(lines)


if __name__ == "__main__":
    for generated_path in write_all_scenarios():
        print(generated_path)
