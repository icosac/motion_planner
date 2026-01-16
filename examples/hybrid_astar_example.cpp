#include <chrono>
#include <fstream>
#include <iostream>
#include <vector>

#include "motion_planner/hybrid_astar.hpp"
#include "motion_planner/shortcutting.hpp"
#include "motion_planner/types.hpp"

struct CircleObstacle {
    double x;
    double y;
    double r;
};

int main() {
    using motion_planner::State;

    // Start/goal in SE(2). Hybrid A* uses motion primitives with curvature.
    const State start{1.0, 1.0, 0.0};
    const State goal{9.0, 9.0, 1.2};

    const double min_x = 0.0;
    const double max_x = 10.0;
    const double min_y = 0.0;
    const double max_y = 10.0;

    // Simple circular obstacles.
    const std::vector<CircleObstacle> obstacles = {
        {4.0, 5.0, 1.1},
        {6.0, 4.0, 1.0},
    };

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

    motion_planner::HybridAStarOptions options;
    options.min_x = min_x;
    options.max_x = max_x;
    options.min_y = min_y;
    options.max_y = max_y;
    options.resolution = 0.25;
    options.theta_bins = 72;
    options.step_size = 0.7;
    options.turning_radius = 1.0;
    options.allow_reverse = true;
    options.goal_tolerance = 0.8;
    options.edge_check_step = 0.1;
    options.max_expansions = 40000;
    options.use_reeds_shepp_goal = true;
    options.reeds_shepp.turning_radius = options.turning_radius;
    options.reeds_shepp.step_size = 0.1;

    // Log outputs for visualization.
    std::ofstream obs_out("hybrid_obstacles.csv");
    obs_out << "x,y,r\n";
    for (const auto &obs : obstacles) {
        obs_out << obs.x << "," << obs.y << "," << obs.r << "\n";
    }

    std::ofstream map_out("hybrid_map.csv");
    map_out << "min_x,max_x,min_y,max_y,resolution,allow_diagonal\n";
    map_out << options.min_x << "," << options.max_x << ","
            << options.min_y << "," << options.max_y << ","
            << options.resolution << ",0\n";

    std::ofstream path_out("hybrid_path.csv");
    path_out << "x,y,theta\n";

    std::ofstream nodes_out("hybrid_nodes.csv");
    nodes_out << "x,y,theta\n";

    auto start_time = std::chrono::steady_clock::now();

    motion_planner::HybridAStar planner(options);
    const auto result = planner.plan_with_nodes(start, goal, distance, is_state_valid);
    const auto &path = result.path;

    if (path.empty()) {
        std::cout << "No path found.\n";
        return 0;
    }

    auto steer = [&](const State &from, const State &to) {
        return motion_planner::reeds_shepp_path(from, to, options.reeds_shepp);
    };

    motion_planner::ShortcutOptions shortcut_options;
    shortcut_options.max_iterations = 200;
    shortcut_options.min_improvement = 1e-3;

    const auto shortened = motion_planner::randomized_shortcut(
        path,
        steer,
        distance,
        is_state_valid,
        shortcut_options);

    std::ofstream shortcut_out("hybrid_path_shortcut.csv");
    shortcut_out << "x,y,theta\n";

    std::cout << "Path found. States: " << path.size() << "\n";
    // for (const auto &state : path) {
    //     std::cout << state.x << ", " << state.y << ", " << state.theta << "\n";
    //     path_out << state.x << "," << state.y << "," << state.theta << "\n";
    // }

    // std::cout << "Shortcut path states: " << shortened.size() << "\n";
    // for (const auto &state : shortened) {
    //     shortcut_out << state.x << "," << state.y << "," << state.theta << "\n";
    // }

    // for (const auto &state : result.nodes) {
    //     nodes_out << state.x << "," << state.y << "," << state.theta << "\n";
    // }

    auto end_time = std::chrono::steady_clock::now();
    const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        end_time - start_time);
    std::cout << "Planning time: " << elapsed_ms.count() << " ms\n";

    return 0;
}
