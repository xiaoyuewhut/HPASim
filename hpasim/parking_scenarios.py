"""OpenDRIVE generation for the HPASim memory parking test map."""

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
    """Straight OpenDRIVE road used as a local reference line."""

    road_id: int
    name: str
    length: float
    x: float
    y: float
    heading: float = 0.0
    lane_width: float = 3.4
    left_lanes: int = 1
    right_lanes: int = 1


@dataclass(frozen=True)
class BoxObject:
    """A rectangular object expressed in a road's local s/t frame."""

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
    road_id: int | None = None


@dataclass(frozen=True)
class ScenarioSpec:
    """Complete parking lot scenario specification."""

    key: str
    name: str
    description: str
    ego_start: tuple[float, float, float]
    target_parking_space: str
    roads: tuple[RoadSpec, ...]
    objects: tuple[BoxObject, ...] = field(default_factory=tuple)


def _box_corners(length: float, width: float) -> list[tuple[float, float]]:
    half_l = length / 2.0
    half_w = width / 2.0
    return [(-half_l, -half_w), (half_l, -half_w), (half_l, half_w), (-half_l, half_w)]


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
    ET.SubElement(
        lane,
        "roadMark",
        sOffset="0.0",
        type="broken",
        weight="standard",
        color="standard",
        width="0.12",
    )


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
        if obj.road_id is not None and obj.road_id != road.road_id:
            continue
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
        if obj.road_id is not None:
            obj_el.set("roadId", str(obj.road_id))
        if obj.object_type == "parkingSpace":
            ET.SubElement(obj_el, "parkingSpace", access="all")
        outline = ET.SubElement(obj_el, "outline")
        for index, (u, v) in enumerate(_box_corners(obj.length, obj.width)):
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


def _parking_space(
    object_id: int,
    road_id: int,
    name: str,
    s: float,
    t: float,
    heading: float,
    subtype: str | None = None,
    width: float = 2.6,
) -> BoxObject:
    fill = {
        "target": "#57c7ff",
        "charging": "#9be7c8",
        "accessible": "#b7d7ff",
        "reserved": "#f4d35e",
    }.get(subtype, "#e6e6e6")
    return BoxObject(
        object_id,
        name,
        "parkingSpace",
        s,
        t,
        5.4,
        width,
        heading,
        subtype=subtype,
        fill=fill,
        road_id=road_id,
    )


def _parking_space_with_size(
    object_id: int,
    road_id: int,
    name: str,
    s: float,
    t: float,
    heading: float,
    length: float,
    width: float,
    subtype: str | None = None,
) -> BoxObject:
    fill = {
        "target": "#57c7ff",
        "charging": "#9be7c8",
        "accessible": "#b7d7ff",
        "reserved": "#f4d35e",
    }.get(subtype, "#e6e6e6")
    return BoxObject(
        object_id,
        name,
        "parkingSpace",
        s,
        t,
        length,
        width,
        heading,
        subtype=subtype,
        fill=fill,
        road_id=road_id,
    )


def _vehicle(
    object_id: int,
    road_id: int,
    name: str,
    s: float,
    t: float,
    heading: float,
    length: float = 4.6,
    width: float = 2.0,
) -> BoxObject:
    return BoxObject(
        object_id,
        name,
        "vehicle",
        s,
        t,
        length,
        width,
        heading,
        fill="#8f8f8f",
        road_id=road_id,
    )


def _parking_bank(
    start_id: int,
    road_id: int,
    prefix: str,
    s_values: tuple[float, ...],
    t: float,
    heading: float,
    target_slot: int | None = None,
    occupied_slots: tuple[int, ...] = (),
    charging_slots: tuple[int, ...] = (),
    accessible_slots: tuple[int, ...] = (),
    reserved_slots: tuple[int, ...] = (),
) -> tuple[BoxObject, ...]:
    objects: list[BoxObject] = []
    for index, s in enumerate(s_values, start=1):
        subtype = None
        width = 2.6
        if index == target_slot:
            subtype = "target"
        elif index in charging_slots:
            subtype = "charging"
        elif index in accessible_slots:
            subtype = "accessible"
            width = 3.2
        elif index in reserved_slots:
            subtype = "reserved"
        objects.append(_parking_space(start_id + index, road_id, f"{prefix}_{index:02d}", s, t, heading, subtype, width))
        if index in occupied_slots:
            objects.append(_vehicle(start_id + 500 + index, road_id, f"{prefix}_vehicle_{index:02d}", s, t, heading))
    return tuple(objects)


def _parallel_parking_bank(
    start_id: int,
    road_id: int,
    prefix: str,
    s_values: tuple[float, ...],
    t: float,
    occupied_slots: tuple[int, ...] = (),
    reserved_slots: tuple[int, ...] = (),
) -> tuple[BoxObject, ...]:
    objects: list[BoxObject] = []
    for index, s in enumerate(s_values, start=1):
        subtype = "reserved" if index in reserved_slots else None
        objects.append(
            _parking_space_with_size(
                start_id + index,
                road_id,
                f"{prefix}_{index:02d}",
                s,
                t,
                0.0,
                6.2,
                2.35,
                subtype,
            )
        )
        if index in occupied_slots:
            objects.append(
                _vehicle(
                    start_id + 500 + index,
                    road_id,
                    f"{prefix}_vehicle_{index:02d}",
                    s,
                    t,
                    0.0,
                    length=4.7,
                    width=2.0,
                )
            )
    return tuple(objects)


def _angled_parking_bank(
    start_id: int,
    road_id: int,
    prefix: str,
    s_values: tuple[float, ...],
    t: float,
    heading: float,
    target_slot: int | None = None,
    occupied_slots: tuple[int, ...] = (),
    charging_slots: tuple[int, ...] = (),
) -> tuple[BoxObject, ...]:
    objects: list[BoxObject] = []
    for index, s in enumerate(s_values, start=1):
        subtype = None
        if index == target_slot:
            subtype = "target"
        elif index in charging_slots:
            subtype = "charging"
        objects.append(_parking_space(start_id + index, road_id, f"{prefix}_{index:02d}", s, t, heading, subtype))
        if index in occupied_slots:
            objects.append(_vehicle(start_id + 500 + index, road_id, f"{prefix}_vehicle_{index:02d}", s, t, heading))
    return tuple(objects)


def _crosswalk(
    start_id: int,
    road_id: int,
    prefix: str,
    s: float,
    offsets: tuple[float, ...],
) -> tuple[BoxObject, ...]:
    return tuple(
        BoxObject(
            start_id + index,
            f"{prefix}_{index:02d}",
            "crosswalk",
            s + offset,
            0.0,
            7.0,
            0.32,
            math.pi / 2,
            fill="#ffffff",
            road_id=road_id,
        )
        for index, offset in enumerate(offsets, start=1)
    )


def _parking_lot_objects() -> tuple[BoxObject, ...]:
    perpendicular_spaces = tuple(
        round(start + index * 2.75, 2)
        for start in (10.0, 39.5, 66.5, 96.5)
        for index in range(5)
    )
    angled_spaces = tuple(
        round(start + index * 4.0, 2)
        for start in (10.0, 38.5, 65.5, 96.0)
        for index in range(5)
    )
    parallel_spaces = (
        10.0,
        17.0,
        24.0,
        40.0,
        47.0,
        54.0,
        66.0,
        73.0,
        80.0,
        96.0,
        103.0,
        110.0,
    )
    objects: list[BoxObject] = [
        BoxObject(1001, "south_outer_wall", "barrier", 60.0, -14.0, 120.0, 0.28, fill="#3f3f3c", road_id=101),
        BoxObject(1002, "north_outer_wall", "barrier", 60.0, 62.0, 120.0, 0.28, fill="#3f3f3c", road_id=101),
        BoxObject(1003, "west_outer_wall", "barrier", 0.0, 24.0, 76.0, 0.28, math.pi / 2, fill="#3f3f3c", road_id=101),
        BoxObject(1004, "east_outer_wall", "barrier", 120.0, 24.0, 76.0, 0.28, math.pi / 2, fill="#3f3f3c", road_id=101),
        BoxObject(1005, "entry_gate", "gate", 10.0, -11.0, 12.0, 0.42, fill="#2f80a7", road_id=101),
        BoxObject(1006, "exit_gate", "gate", 110.0, 59.0, 12.0, 0.42, fill="#2f80a7", road_id=101),
        BoxObject(1007, "center_island_west", "island", 39.0, 14.2, 16.0, 1.8, fill="#8dbb75", road_id=102),
        BoxObject(1008, "center_island_east", "island", 81.0, 14.2, 16.0, 1.8, fill="#8dbb75", road_id=102),
        BoxObject(1009, "lower_island_west", "island", 39.0, -14.2, 16.0, 1.8, fill="#8dbb75", road_id=102),
        BoxObject(1010, "lower_island_east", "island", 81.0, -14.2, 16.0, 1.8, fill="#8dbb75", road_id=102),
        BoxObject(1011, "delivery_vehicle_block", "vehicle", 63.0, -0.8, 6.5, 2.35, fill="#7a7a74", road_id=102),
        BoxObject(1012, "work_zone_block", "barrier", 71.0, -3.0, 3.2, 1.4, math.radians(18.0), fill="#f2a12b", road_id=102),
        BoxObject(1013, "entry_speed_bump", "barrier", 36.0, 0.0, 7.2, 0.22, math.pi / 2, fill="#e6c85c", road_id=101),
        BoxObject(1014, "exit_speed_bump", "barrier", 84.0, 0.0, 7.2, 0.22, math.pi / 2, fill="#e6c85c", road_id=103),
    ]
    objects.extend(_parallel_parking_bank(2000, 101, "south_parallel_row", parallel_spaces, -8.4, occupied_slots=(2, 5, 8), reserved_slots=(12,)))
    objects.extend(_parking_bank(3000, 101, "south_perpendicular_row", perpendicular_spaces, 7.8, math.pi / 2, occupied_slots=(4, 8, 14, 18), accessible_slots=(1,)))
    objects.extend(_angled_parking_bank(4000, 102, "central_lower_angled_row", angled_spaces, -7.2, math.radians(-60.0), occupied_slots=(2, 6, 10, 16)))
    objects.extend(_angled_parking_bank(5000, 102, "central_upper_angled_row", angled_spaces, 7.2, math.radians(60.0), target_slot=13, occupied_slots=(3, 7, 17), charging_slots=(19, 20)))
    objects.extend(_parking_bank(6000, 103, "north_perpendicular_row", perpendicular_spaces, -7.8, -math.pi / 2, occupied_slots=(5, 9, 13, 19), reserved_slots=(2,)))
    objects.extend(_parallel_parking_bank(7000, 103, "north_parallel_row", parallel_spaces, 8.4, occupied_slots=(3, 7, 11)))
    objects.extend(_crosswalk(8000, 102, "west_crosswalk", 10.0, (-0.8, -0.4, 0.0, 0.4, 0.8)))
    objects.extend(_crosswalk(8010, 102, "east_crosswalk", 110.0, (-0.8, -0.4, 0.0, 0.4, 0.8)))
    return tuple(objects)


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
        north="80.0",
        south="-40.0",
        east="130.0",
        west="-10.0",
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
        key="parking_lot_full",
        name="Full memory parking lot",
        description=(
            "A complete parking lot map built as a single end-to-end memory parking "
            "scenario with entry, exit, three drive aisles, cross aisles, dense "
            "parking rows, occupied vehicles, islands, crossings, speed bumps, and "
            "one target parking bay."
        ),
        ego_start=(8.0, -1.7, 0.0),
        target_parking_space="central_upper_angled_row_13",
        roads=(
            RoadSpec(road_id=101, name="entry_drive", length=120.0, x=0.0, y=0.0, lane_width=3.4),
            RoadSpec(road_id=102, name="memory_drive", length=120.0, x=0.0, y=26.0, lane_width=3.4),
            RoadSpec(road_id=103, name="exit_drive", length=120.0, x=0.0, y=52.0, lane_width=3.4),
            RoadSpec(road_id=104, name="west_cross_aisle", length=60.0, x=33.0, y=-4.0, heading=math.pi / 2, lane_width=3.0),
            RoadSpec(road_id=105, name="middle_cross_aisle", length=60.0, x=60.0, y=-4.0, heading=math.pi / 2, lane_width=3.0),
            RoadSpec(road_id=106, name="east_cross_aisle", length=60.0, x=87.0, y=-4.0, heading=math.pi / 2, lane_width=3.0),
        ),
        objects=_parking_lot_objects(),
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
