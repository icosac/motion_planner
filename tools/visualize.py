#!/usr/bin/env python3

import subprocess
import sys


def main():
    if len(sys.argv) != 2:
        print("Usage: python3 tools/visualize.py <rrt|hybrid>")
        sys.exit(1)

    algorithm = sys.argv[1].lower()
    if algorithm == "rrt":
        cmd = [
            sys.executable,
            "tools/plot_rrt.py",
            "--tree",
            "rrt_tree.csv",
            "--obstacles",
            "rrt_obstacles.csv",
            "--path",
            "rrt_path.csv",
        ]
    elif algorithm == "hybrid":
        cmd = [
            sys.executable,
            "tools/plot_grid_astar.py",
            "--path",
            "hybrid_path.csv",
            "--obstacles",
            "hybrid_obstacles.csv",
            "--map",
            "hybrid_map.csv",
            "--nodes",
            "hybrid_nodes.csv",
        ]
    else:
        print("Usage: python3 tools/visualize.py <rrt|hybrid>")
        sys.exit(1)

    subprocess.run(cmd, check=False)


if __name__ == "__main__":
    main()
        
