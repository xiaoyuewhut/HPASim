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
    ax.add_patch(Polygon(corners, closed=True, facecolor="#f6f6f6", edgecolor="#222222", linewidth=1.0))

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
        ax.plot([start[0], end[0]], [start[1], end[1]], color="#777777", linewidth=0.8, linestyle="--" if t else "-")


def _object_style(obj: ET.Element) -> tuple[str, str, float]:
    obj_type = obj.get("type", "")
    fill = obj.get("fill")
    if fill:
        return fill, "#222222", 1.0
    if obj_type == "parkingSpace":
        return "#e6e6e6", "#268bd2", 1.2
    if obj_type == "vehicle":
        return "#8f8f8f", "#333333", 1.0
    if obj_type in {"barrier", "curbstone"}:
        return "#555555", "#222222", 1.0
    return "#dddddd", "#333333", 1.0


def plot_opendrive(path: Path, output: Path | None = None, show: bool = False) -> Path | None:
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
    header = root.find("header")

    fig, ax = plt.subplots(figsize=(10, 6), constrained_layout=True)
    ax.set_aspect("equal", adjustable="box")
    ax.set_title(header.get("name") if header is not None else path.stem)
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.grid(True, color="#dddddd", linewidth=0.6)

    all_x: list[float] = []
    all_y: list[float] = []
    for road in root.findall("road"):
        frame = _road_frame(road)
        _draw_lane_band(ax, frame)
        for s, t in [(0.0, -sum(frame.lane_widths_right)), (frame.length, sum(frame.lane_widths_left))]:
            x, y = frame.st_to_xy(s, t)
            all_x.append(x)
            all_y.append(y)

        for obj in road.findall("./objects/object"):
            corners = obj.findall("./outline/cornerLocal")
            if not corners:
                continue
            points = [frame.st_to_xy(*_local_corner_to_st(obj, corner)) for corner in corners]
            face, edge, width = _object_style(obj)
            ax.add_patch(Polygon(points, closed=True, facecolor=face, edgecolor=edge, linewidth=width, alpha=0.88))
            center = frame.st_to_xy(_float_attr(obj, "s"), _float_attr(obj, "t"))
            ax.text(center[0], center[1], obj.get("name", ""), fontsize=7, ha="center", va="center")
            all_x.extend(point[0] for point in points)
            all_y.extend(point[1] for point in points)

    if all_x and all_y:
        margin = 4.0
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
