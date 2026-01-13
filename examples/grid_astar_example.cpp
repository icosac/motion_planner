#include <fstream>
#include <iostream>
#include <vector>

#include "motion_planner/grid_astar.hpp"
#include "motion_planner/types.hpp"

struct CircleObstacle {
    double x;
    double y;
    double r;
};

int main() {
    using motion_planner::State;

    // Start/goal in SE(2). Grid A* will infer headings from grid moves.
    const State start{1.0, 1.0, 0.0};
    const State goal{9.0, 9.0, 1.2};

    const double min_x = 0.0;
    const double max_x = 10.0;
    const double min_y = 0.0;
    const double max_y = 10.0;

    // Simple circular obstacles.
    const std::vector<CircleObstacle> obstacles = {
        {4.0, 5.0, 1.0},
        {6.0, 4.0, 1.0},
    };

    // Log outputs for visualization.
    std::ofstream obs_out("grid_obstacles.csv");
    obs_out << "x,y,r\n";
    for (const auto &obs : obstacles) {
        obs_out << obs.x << "," << obs.y << "," << obs.r << "\n";
    }

    std::ofstream map_out("grid_map.csv");
    map_out << "min_x,max_x,min_y,max_y,resolution,allow_diagonal\n";

    // Distance metric for A* priority (SE(2) heuristic).
    auto distance = [&](const State &a, const State &b) {
        return motion_planner::se2_distance(a, b);
    };

    // Collision oracle for a single state.
    auto is_state_valid = [&](const State &state) {
        if (state.x < min_x || state.x > max_x || state.y < min_y || state.y > max_y) {
            return false;
        }
        for (const auto &obs : obstacles) {
            const double dx = state.x - obs.x;
            const double dy = state.y - obs.y;
            if ((dx * dx + dy * dy) <= (obs.r * obs.r)) {
                return false;
            }
        }
        return true;
    };

    motion_planner::GridAStarOptions options;
    options.min_x = min_x;
    options.max_x = max_x;
    options.min_y = min_y;
    options.max_y = max_y;
    options.resolution = 0.25;
    options.goal_tolerance = 0.8;
    options.allow_diagonal = true;
    // Sample along edges to avoid cutting through obstacles.
    options.edge_check_step = 0.25;
    options.max_expansions = 20000;

    map_out << options.min_x << "," << options.max_x << ","
            << options.min_y << "," << options.max_y << ","
            << options.resolution << "," << (options.allow_diagonal ? 1 : 0) << "\n";

    std::ofstream path_out("grid_path.csv");
    path_out << "x,y,theta\n";

    motion_planner::GridAStar planner(options);
    const auto path = planner.plan(start, goal, distance, is_state_valid);

    if (path.empty()) {
        std::cout << "No path found.\n";
        return 0;
    }

    std::cout << "Path found. Waypoints: " << path.size() << "\n";
    for (const auto &state : path) {
        std::cout << state.x << ", " << state.y << ", " << state.theta << "\n";
        path_out << state.x << "," << state.y << "," << state.theta << "\n";
    }

    return 0;
}
