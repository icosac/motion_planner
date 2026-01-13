#!/usr/bin/env python3
import argparse
import csv
import math
from typing import List, Optional, Tuple

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.collections import LineCollection
from matplotlib.patches import Circle


def load_path(path: str) -> List[Tuple[float, float, float]]:
    points = []
    with open(path, newline="") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            points.append((float(row["x"]), float(row["y"]), float(row["theta"])))
    return points


def load_obstacles(path: Optional[str]) -> List[Tuple[float, float, float]]:
    obstacles = []
    if path is None:
        return obstacles
    with open(path, newline="") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            obstacles.append((float(row["x"]), float(row["y"]), float(row["r"])))
    return obstacles


def load_map(path: str):
    with open(path, newline="") as handle:
        reader = csv.DictReader(handle)
        row = next(reader, None)
        if row is None:
            raise ValueError("map file is empty")
        return (
            float(row["min_x"]),
            float(row["max_x"]),
            float(row["min_y"]),
            float(row["max_y"]),
            float(row["resolution"]),
            int(row.get("allow_diagonal", "0")),
        )


def build_grid(min_x: float, max_x: float, min_y: float, max_y: float, resolution: float):
    if resolution <= 0.0:
        raise ValueError("resolution must be positive")
    width = int(math.floor((max_x - min_x) / resolution)) + 1
    height = int(math.floor((max_y - min_y) / resolution)) + 1
    xs = np.linspace(min_x, min_x + (width - 1) * resolution, width)
    ys = np.linspace(min_y, min_y + (height - 1) * resolution, height)
    return xs, ys


def segment_hits_obstacles(
    start: Tuple[float, float],
    end: Tuple[float, float],
    obstacles: List[Tuple[float, float, float]],
) -> bool:
    x0, y0 = start
    x1, y1 = end
    dx = x1 - x0
    dy = y1 - y0
    denom = dx * dx + dy * dy
    for ox, oy, r in obstacles:
        if denom <= 1e-12:
            dist2 = (ox - x0) ** 2 + (oy - y0) ** 2
        else:
            t = ((ox - x0) * dx + (oy - y0) * dy) / denom
            t = min(1.0, max(0.0, t))
            cx = x0 + t * dx
            cy = y0 + t * dy
            dist2 = (ox - cx) ** 2 + (oy - cy) ** 2
        if dist2 <= r * r:
            return True
    return False


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Plot grid A* obstacles, path, and grid vertices/edges."
    )
    parser.add_argument("--path", required=True, help="Path to grid_path.csv")
    parser.add_argument("--obstacles", help="Path to grid_obstacles.csv")
    parser.add_argument("--map", help="Path to grid_map.csv")
    parser.add_argument("--min-x", type=float, help="Grid min x (if --map not provided)")
    parser.add_argument("--max-x", type=float, help="Grid max x (if --map not provided)")
    parser.add_argument("--min-y", type=float, help="Grid min y (if --map not provided)")
    parser.add_argument("--max-y", type=float, help="Grid max y (if --map not provided)")
    parser.add_argument("--resolution", type=float, help="Grid resolution (if --map not provided)")
    parser.add_argument("--no-vertices", action="store_true", help="Disable grid vertices")
    parser.add_argument("--no-edges", action="store_true", help="Disable grid edges")
    parser.add_argument("--nodes", help="Path to nodes CSV to highlight lattice nodes")
    args = parser.parse_args()

    path = load_path(args.path)
    obstacles = load_obstacles(args.obstacles)

    if args.map:
        min_x, max_x, min_y, max_y, resolution, allow_diagonal = load_map(args.map)
    else:
        missing = [
            name
            for name, value in [
                ("--min-x", args.min_x),
                ("--max-x", args.max_x),
                ("--min-y", args.min_y),
                ("--max-y", args.max_y),
                ("--resolution", args.resolution),
            ]
            if value is None
        ]
        if missing:
            raise SystemExit(f"Missing grid arguments: {', '.join(missing)}")
        min_x = args.min_x
        max_x = args.max_x
        min_y = args.min_y
        max_y = args.max_y
        resolution = args.resolution
        allow_diagonal = 0

    xs, ys = build_grid(min_x, max_x, min_y, max_y, resolution)

    fig, ax = plt.subplots(figsize=(7, 7))
    ax.set_aspect("equal", adjustable="box")
    ax.set_title("Grid A* Path")

    xv, yv = np.meshgrid(xs, ys)
    valid_mask = np.ones_like(xv, dtype=bool)
    for obs_x, obs_y, obs_r in obstacles:
        dist2 = (xv - obs_x) ** 2 + (yv - obs_y) ** 2
        valid_mask &= dist2 > obs_r * obs_r

    if not args.no_edges:
        segments = []
        height, width = valid_mask.shape
        for j in range(height):
            for i in range(width):
                if not valid_mask[j, i]:
                    continue
                if i + 1 < width and valid_mask[j, i + 1]:
                    start = (xs[i], ys[j])
                    end = (xs[i + 1], ys[j])
                    if not segment_hits_obstacles(start, end, obstacles):
                        segments.append((start, end))
                if j + 1 < height and valid_mask[j + 1, i]:
                    start = (xs[i], ys[j])
                    end = (xs[i], ys[j + 1])
                    if not segment_hits_obstacles(start, end, obstacles):
                        segments.append((start, end))
                if allow_diagonal:
                    if i + 1 < width and j + 1 < height and valid_mask[j + 1, i + 1]:
                        start = (xs[i], ys[j])
                        end = (xs[i + 1], ys[j + 1])
                        if not segment_hits_obstacles(start, end, obstacles):
                            segments.append((start, end))
                    if i + 1 < width and j - 1 >= 0 and valid_mask[j - 1, i + 1]:
                        start = (xs[i], ys[j])
                        end = (xs[i + 1], ys[j - 1])
                        if not segment_hits_obstacles(start, end, obstacles):
                            segments.append((start, end))
        grid_lines = LineCollection(segments, colors="#cccccc", linewidths=0.6, alpha=0.6)
        ax.add_collection(grid_lines)

    if not args.no_vertices:
        ax.scatter(xv[valid_mask], yv[valid_mask], s=6, c="#999999", alpha=0.5, zorder=1)

    for obs_x, obs_y, obs_r in obstacles:
        ax.add_patch(Circle((obs_x, obs_y), obs_r, color="#666666", alpha=0.4, zorder=2))

    if path:
        path_x = [p[0] for p in path]
        path_y = [p[1] for p in path]
        ax.plot(path_x, path_y, color="#ff7f0e", linewidth=2.0, label="path", zorder=3)
        ax.plot(path_x[0], path_y[0], "go", markersize=6, label="start", zorder=4)
        ax.plot(path_x[-1], path_y[-1], "r*", markersize=8, label="goal", zorder=4)

    if args.nodes:
        node_points = load_path(args.nodes)
        if node_points:
            node_x = [p[0] for p in node_points]
            node_y = [p[1] for p in node_points]
            ax.scatter(node_x, node_y, s=24, c="#1f77b4", alpha=0.7, zorder=3, label="path nodes")

    margin = 0.5
    ax.set_xlim(min_x - margin, max_x + margin)
    ax.set_ylim(min_y - margin, max_y + margin)
    ax.legend(loc="upper right")
    plt.show()


if __name__ == "__main__":
    main()
