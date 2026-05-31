"""Matplotlib renderer for the straight-road parking OpenDRIVE scenarios."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import argparse
import math
import os
import subprocess
import sys
import xml.etree.ElementTree as ET


PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_MAP_DIR = PROJECT_ROOT / "maps" / "opendrive"
DEFAULT_OUTPUT_DIR = PROJECT_ROOT / "outputs"
VENV_PYTHON = PROJECT_ROOT / ".venv" / "Scripts" / "python.exe"
os.environ.setdefault("MPLCONFIGDIR", str(PROJECT_ROOT / ".matplotlib"))


@dataclass(frozen=True)
class RoadFrame:
    x: float
    y: float
    heading: float
    length: float
    lane_widths_left: tuple[float, ...]
    lane_widths_right: tuple[float, ...]

    def st_to_xy(self, s: float, t: float) -> tuple[float, float]:
        cos_h = math.cos(self.heading)
        sin_h = math.sin(self.heading)
        return (
            self.x + s * cos_h - t * sin_h,
            self.y + s * sin_h + t * cos_h,
        )


@dataclass(frozen=True)
class RoadRender:
    element: ET.Element
    frame: RoadFrame


def _float_attr(element: ET.Element, name: str, default: float = 0.0) -> float:
    value = element.get(name)
    return default if value is None else float(value)


def _road_frame(road: ET.Element) -> RoadFrame:
    geometry = road.find("./planView/geometry")
    if geometry is None or geometry.find("line") is None:
        raise ValueError("Only straight line OpenDRIVE geometry is supported by this plotter.")

    left_widths = tuple(
        _float_attr(width, "a")
        for width in road.findall("./lanes/laneSection/left/lane/width")
    )
    right_widths = tuple(
        _float_attr(width, "a")
        for width in road.findall("./lanes/laneSection/right/lane/width")
    )
    return RoadFrame(
        x=_float_attr(geometry, "x"),
        y=_float_attr(geometry, "y"),
        heading=_float_attr(geometry, "hdg"),
        length=_float_attr(geometry, "length"),
        lane_widths_left=left_widths,
        lane_widths_right=right_widths,
    )


def _local_corner_to_st(obj: ET.Element, corner: ET.Element) -> tuple[float, float]:
    obj_s = _float_attr(obj, "s")
    obj_t = _float_attr(obj, "t")
    obj_hdg = _float_attr(obj, "hdg")
    u = _float_attr(corner, "u")
    v = _float_attr(corner, "v")
    cos_h = math.cos(obj_hdg)
    sin_h = math.sin(obj_hdg)
    return (
        obj_s + u * cos_h - v * sin_h,
        obj_t + u * sin_h + v * cos_h,
    )


def _draw_lane_band(ax: plt.Axes, frame: RoadFrame) -> None:
    from matplotlib.patches import Polygon

    left_total = sum(frame.lane_widths_left)
    right_total = sum(frame.lane_widths_right)
    corners = [
        frame.st_to_xy(0.0, -right_total),
        frame.st_to_xy(frame.length, -right_total),
        frame.st_to_xy(frame.length, left_total),
        frame.st_to_xy(0.0, left_total),
    ]
    ax.add_patch(
        Polygon(
            corners,
            closed=True,
            facecolor="#f2f3f0",
            edgecolor="#a1a198",
            linewidth=0.65,
            zorder=1,
        )
    )

    lane_offsets = [0.0]
    running = 0.0
    for width in frame.lane_widths_left:
        running += width
        lane_offsets.append(running)
    running = 0.0
    for width in frame.lane_widths_right:
        running -= width
        lane_offsets.append(running)

    for t in lane_offsets:
        start = frame.st_to_xy(0.0, t)
        end = frame.st_to_xy(frame.length, t)
        ax.plot(
            [start[0], end[0]],
            [start[1], end[1]],
            color="#c5c5bc",
            linewidth=0.55,
            linestyle="--" if t else "-",
            zorder=2,
        )


def _object_style(obj: ET.Element) -> tuple[str, str, float]:
    obj_type = obj.get("type", "")
    subtype = obj.get("subtype", "")
    fill = obj.get("fill")
    if subtype == "target":
        return "#4fb3ff", "#0969a8", 1.5
    if subtype == "charging":
        return "#9fe3c2", "#3a8b66", 1.0
    if subtype == "accessible":
        return "#b9d6ff", "#5686c6", 1.0
    if subtype == "reserved":
        return "#efd66f", "#947b20", 1.0
    if obj_type == "gate":
        return fill or "#2f80a7", "#204b61", 1.2
    if obj_type == "ramp":
        return fill or "#dedbd1", "#9d998f", 0.9
    if obj_type == "column":
        return fill or "#6e6e68", "#333330", 1.2
    if obj_type == "pedestrianWalkway":
        return fill or "#f7f4df", "#d7d0a8", 0.55
    if obj_type == "parkingSpace":
        return "#fafaf7", "#8e8e86", 0.85
    if obj_type == "vehicle":
        return fill or "#7a7a74", "#3d3d3a", 1.0
    if obj_type in {"barrier", "curbstone"}:
        return fill or "#555555", "#3a3a36", 1.0
    if fill:
        return fill, "#4a4a46", 0.9
    return "#dddddd", "#4a4a46", 0.9


def _object_zorder(obj: ET.Element) -> int:
    obj_type = obj.get("type", "")
    if obj_type in {"pedestrianWalkway", "ramp"}:
        return 3
    if obj_type == "parkingSpace":
        return 4
    if obj_type == "gate":
        return 5
    if obj_type == "vehicle":
        return 6
    if obj_type in {"barrier", "curbstone", "column"}:
        return 7
    return 5


def plot_opendrive(
    path: Path,
    output: Path | None = None,
    show: bool = False,
) -> Path | None:
    try:
        if not show:
            import matplotlib

            matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ModuleNotFoundError as exc:
        if exc.name == "matplotlib":
            raise SystemExit(
                "matplotlib is not installed in this Python environment. "
                "Run with .\\.venv\\Scripts\\python.exe or install dependencies first."
            ) from exc
        raise
    from matplotlib.patches import Polygon

    tree = ET.parse(path)
    root = tree.getroot()

    fig, ax = plt.subplots(figsize=(12, 8), constrained_layout=True)
    fig.patch.set_facecolor("#f7f7f3")
    ax.set_facecolor("#f7f7f3")
    ax.set_aspect("equal", adjustable="box")
    ax.grid(False)
    ax.tick_params(left=False, bottom=False, labelbottom=False, labelleft=False)
    for spine in ax.spines.values():
        spine.set_visible(False)

    all_x: list[float] = []
    all_y: list[float] = []
    roads = [RoadRender(road, _road_frame(road)) for road in root.findall("road")]

    for road_render in roads:
        frame = road_render.frame
        _draw_lane_band(ax, frame)
        for s, t in [(0.0, -sum(frame.lane_widths_right)), (frame.length, sum(frame.lane_widths_left))]:
            x, y = frame.st_to_xy(s, t)
            all_x.append(x)
            all_y.append(y)

    for road_render in roads:
        frame = road_render.frame
        for obj in road_render.element.findall("./objects/object"):
            corners = obj.findall("./outline/cornerLocal")
            if not corners:
                continue
            points = [frame.st_to_xy(*_local_corner_to_st(obj, corner)) for corner in corners]
            face, edge, width = _object_style(obj)
            ax.add_patch(
                Polygon(
                    points,
                    closed=True,
                    facecolor=face,
                    edgecolor=edge,
                    linewidth=width,
                    alpha=0.95,
                    zorder=_object_zorder(obj),
                )
            )
            all_x.extend(point[0] for point in points)
            all_y.extend(point[1] for point in points)

    if all_x and all_y:
        margin = 5.0
        ax.set_xlim(min(all_x) - margin, max(all_x) + margin)
        ax.set_ylim(min(all_y) - margin, max(all_y) + margin)

    if output:
        output.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(output, dpi=160)
    if show:
        plt.show()
    plt.close(fig)
    return output


def _restart_with_venv_python_if_needed() -> None:
    try:
        import matplotlib  # noqa: F401
    except ModuleNotFoundError as exc:
        if exc.name != "matplotlib" or not VENV_PYTHON.exists() or Path(sys.executable) == VENV_PYTHON:
            return
        completed = subprocess.run([str(VENV_PYTHON), *sys.argv])
        raise SystemExit(completed.returncode) from exc


def plot_all_opendrive_maps(
    map_dir: Path = DEFAULT_MAP_DIR,
    output_dir: Path = DEFAULT_OUTPUT_DIR,
    show: bool = False,
) -> list[Path | None]:
    paths = sorted(map_dir.glob("*.xodr"))
    if not paths:
        raise FileNotFoundError(f"No .xodr maps found in {map_dir}")
    return [
        plot_opendrive(path, output_dir / f"{path.stem}.png", show)
        for path in paths
    ]


def main() -> None:
    _restart_with_venv_python_if_needed()

    parser = argparse.ArgumentParser(description="Plot HPASim OpenDRIVE parking maps.")
    parser.add_argument(
        "xodr",
        type=Path,
        nargs="?",
        help="Path to an OpenDRIVE .xodr file. If omitted, all maps are rendered.",
    )
    parser.add_argument("--output", "-o", type=Path, help="Output image path.")
    parser.add_argument("--show", action="store_true", help="Show an interactive matplotlib window.")
    args = parser.parse_args()
    if args.xodr is None:
        for output in plot_all_opendrive_maps(show=args.show):
            if output:
                print(output)
        return

    output = args.output or DEFAULT_OUTPUT_DIR / f"{args.xodr.stem}.png"
    rendered = plot_opendrive(args.xodr, output, args.show)
    if rendered:
        print(rendered)


if __name__ == "__main__":
    main()
