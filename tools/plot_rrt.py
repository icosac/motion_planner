#!/usr/bin/env python3
import argparse
import csv
import math
from typing import Dict, List, Tuple

import matplotlib.pyplot as plt
import numpy as np
from matplotlib import animation
from matplotlib.collections import LineCollection
from matplotlib.patches import Circle


def parse_float(value: str) -> float:
    if value is None or value == "":
        return float("nan")
    return float(value)


def load_events(path: str):
    events = []
    root = None
    goal = None
    xs = []
    ys = []

    with open(path, newline="") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            print(row)
            event_type = row["type"].strip().lower()
            child = (parse_float(row["child_x"]), parse_float(row["child_y"]))
            parent = (parse_float(row["parent_x"]), parse_float(row["parent_y"]))
            child_index = int(row.get("child_index", -1))
            parent_index = int(row.get("parent_index", -1))

            if event_type == "root":
                root = child
            else:
                events.append(
                    {
                        "type": event_type,
                        "parent": parent,
                        "child": child,
                        "child_index": child_index,
                        "parent_index": parent_index,
                    }
                )

            xs.append(child[0])
            ys.append(child[1])
            if not math.isnan(parent[0]):
                xs.append(parent[0])
                ys.append(parent[1])

            if event_type == "goal" and goal is None:
                goal = child

    return events, root, goal, xs, ys


def load_obstacles(path: str):
    obstacles = []
    if path is None:
        return obstacles

    with open(path, newline="") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            obstacles.append(
                (float(row["x"]), float(row["y"]), float(row["r"]))
            )
    return obstacles


def main():
    parser = argparse.ArgumentParser(description="Animate RRT/RRT* growth from CSV logs.")
    parser.add_argument("--tree", required=True, help="Path to rrt_tree.csv")
    parser.add_argument("--obstacles", help="Path to rrt_obstacles.csv")
    parser.add_argument("--path", help="Path to rrt_path.csv for final trajectory")
    parser.add_argument("--interval", type=float, default=20.0, help="Animation interval in ms")
    parser.add_argument("--interval-us", type=float, help="Animation interval in microseconds")
    parser.add_argument("--interval-ns", type=float, help="Animation interval in nanoseconds")
    parser.add_argument("--blit", action="store_true", help="Enable blitting for faster rendering")
    args = parser.parse_args()

    events, root, goal, xs, ys = load_events(args.tree)
    obstacles = load_obstacles(args.obstacles)

    interval_ms = args.interval
    if args.interval_us is not None:
        interval_ms = args.interval_us / 1000.0
    if args.interval_ns is not None:
        interval_ms = args.interval_ns / 1_000_000.0

    if not events and root is None:
        print("No events to display.")
        return

    fig, ax = plt.subplots(figsize=(7, 7))
    ax.set_aspect("equal", adjustable="box")
    ax.set_title("RRT Tree Growth")

    if xs and ys:
        margin = 0.5
        ax.set_xlim(min(xs) - margin, max(xs) + margin)
        ax.set_ylim(min(ys) - margin, max(ys) + margin)

    for obs_x, obs_y, obs_r in obstacles:
        ax.add_patch(Circle((obs_x, obs_y), obs_r, color="#666", alpha=0.4))

    if root is not None:
        ax.plot(root[0], root[1], "go", markersize=6, label="start")
    if goal is not None:
        ax.plot(goal[0], goal[1], "r*", markersize=8, label="goal")

    lc = LineCollection([], colors="#1f77b4", linewidths=0.8, alpha=0.8)
    ax.add_collection(lc)

    node_scatter = ax.scatter([], [], s=6, c="#1f77b4", alpha=0.6)
    path_line, = ax.plot([], [], color="#ff7f0e", linewidth=2.0, alpha=0.9, label="path")

    if args.path:
        path_xs = []
        path_ys = []
        with open(args.path, newline="") as handle:
            reader = csv.DictReader(handle)
            for row in reader:
                path_xs.append(float(row["x"]))
                path_ys.append(float(row["y"]))
        if path_xs:
            path_line.set_data(path_xs, path_ys)

    edges: List[Tuple[Tuple[float, float], Tuple[float, float]]] = []
    edge_index: Dict[int, int] = {}
    nodes: List[Tuple[float, float]] = []

    def init():
        edges.clear()
        edge_index.clear()
        nodes.clear()
        lc.set_segments([])
        node_scatter.set_offsets(np.empty((0, 2)))
        return lc, node_scatter, path_line

    def apply_event(event):
        parent = event["parent"]
        child = event["child"]
        child_index = event["child_index"]
        event_type = event["type"]

        if event_type in ("add", "goal"):
            edges.append((parent, child))
            edge_index[child_index] = len(edges) - 1
            nodes.append(child)
        elif event_type == "rewire":
            idx = edge_index.get(child_index)
            if idx is None:
                edges.append((parent, child))
                edge_index[child_index] = len(edges) - 1
            else:
                edges[idx] = (parent, child)

    def animate(frame):
        apply_event(events[frame])
        lc.set_segments(edges)
        if nodes:
            node_scatter.set_offsets(np.asarray(nodes))
        else:
            node_scatter.set_offsets(np.empty((0, 2)))
        return lc, node_scatter, path_line

    ani = animation.FuncAnimation(
        fig,
        animate,
        init_func=init,
        frames=len(events),
        interval=interval_ms,
        blit=args.blit,
        repeat=False,
    )

    if root is not None or goal is not None:
        ax.legend(loc="upper right")

    plt.show()


if __name__ == "__main__":
    main()
