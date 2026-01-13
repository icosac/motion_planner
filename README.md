# Motion Planner

A small C++17 motion-planning library for SE(2) that provides RRT and RRT* with a customizable steering function. The default example uses a Reeds-Shepp steering function with a simple collision oracle.

## Features

- RRT and RRT* planners for 2D Euclidean space with heading.
- User-provided sampling, distance, steering, and collision checks.
- Reeds-Shepp steering with forward and reverse segments.
- Static library build via CMake.

## Build

```sh
cmake -S . -B build
cmake --build build
```

## Example

Run the example after building:

```sh
./build/rrt_reeds_shepp_example
```

The example prints a sequence of states (x, y, theta) for the planned path.

## Visualization

The example also writes `rrt_tree.csv` and `rrt_obstacles.csv` in the project root. Use the helper script to animate tree growth:

```sh
python3 tools/plot_rrt.py --tree rrt_tree.csv --obstacles rrt_obstacles.csv
```

The script requires matplotlib (`pip install matplotlib`).

## Basic Usage

```cpp
#include "motion_planner/rrt_star.hpp"
#include "motion_planner/mpdp_reeds_shepp.hpp"
#include "motion_planner/samplers.hpp"
#include "motion_planner/types.hpp"

motion_planner::RRTStar planner(options);

motion_planner::UniformSamplerConfig sampler_cfg;
sampler_cfg.min_x = 0.0;
sampler_cfg.max_x = 10.0;
sampler_cfg.min_y = 0.0;
sampler_cfg.max_y = 10.0;
auto sampler = motion_planner::make_uniform_sampler(sampler_cfg);
auto distance = [&](const motion_planner::State &a, const motion_planner::State &b) {
    return motion_planner::se2_distance(a, b);
};
auto is_state_valid = [&](const motion_planner::State &state) { return true; };

auto steer = [&](const motion_planner::State &from, const motion_planner::State &to) {
    return motion_planner::reeds_shepp_path(from, to, rs_options);
};

auto path = planner.plan(start, goal, sampler, distance, steer, is_state_valid);
```

## Notes

- `reeds_shepp_path` uses the MPDP Reeds-Shepp implementation under the hood. You can replace it with your own steering function if you have a specialized model.
- Collision checking is handled via a user-supplied oracle on individual states.
