#include <chrono>
#include <fstream>
#include <iostream>
#include <random>
#include <string>
#include <vector>

#include "motion_planner/mpdp_reeds_shepp.hpp"
#include "motion_planner/rrt.hpp"
#include "motion_planner/rrt_star.hpp"
#include "motion_planner/samplers.hpp"
#include "motion_planner/types.hpp"
#include "motion_planner/visualization.hpp"

struct CircleObstacle {
    double x;
    double y;
    double r;
};

int main(int argc, char **argv) {
    using motion_planner::State;

    // Parse planner selection flags.
    bool use_rrt_star = true;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--rrt") {
            use_rrt_star = false;
        } else if (arg == "--rrt-star" || arg == "--rrt*") {
            use_rrt_star = true;
        } else if (arg == "--help") {
            std::cout << "Usage: rrt_reeds_shepp_example [--rrt|--rrt-star]\n";
            return 0;
        }
    }

    // Start/goal in SE(2).
    const State start{1.0, 1.0, 0.0};
    const State goal{9.0, 9.0, 1.2};

    const double min_x = 0.0;
    const double max_x = 10.0;
    const double min_y = 0.0;
    const double max_y = 10.0;

    // Simple obstacle map (circle list) and log files for visualization.
    const std::vector<CircleObstacle> obstacles = {
        {4.0, 5.0, 1.0},
        {6.0, 4.0, 1.0},
    };

    std::ofstream tree_out("rrt_tree.csv");
    tree_out << "iter,type,parent_x,parent_y,parent_theta,child_x,child_y,child_theta,cost,parent_index,child_index\n";
    tree_out << -1 << ",root," << start.x << "," << start.y << "," << start.theta << ","
             << start.x << "," << start.y << "," << start.theta << ",0,-1,0\n";

    std::ofstream obs_out("rrt_obstacles.csv");
    obs_out << "x,y,r\n";
    for (const auto &obs : obstacles) {
        obs_out << obs.x << "," << obs.y << "," << obs.r << "\n";
    }

    std::ofstream path_out("rrt_path.csv");
    path_out << "x,y,theta\n";

    // Map planner events to CSV-friendly strings.
    auto event_type_to_string = [](motion_planner::VisualizationEventType type) {
        switch (type) {
        case motion_planner::VisualizationEventType::Add:
            return "add";
        case motion_planner::VisualizationEventType::Rewire:
            return "rewire";
        case motion_planner::VisualizationEventType::Goal:
            return "goal";
        }
        return "add";
    };

    // Shared planner parameters for both RRT and RRT*.
    const double step_size = 0.7;
    const double goal_bias = 0.1;
    const int max_iterations = 3000;
    const double goal_tolerance = 0.8;
    const double neighbor_radius = 1.5;

    // Stream updates for plotting and periodic progress output.
    auto visualization_sink = [&](const motion_planner::VisualizationEvent &event) {
        tree_out << event.iteration << "," << event_type_to_string(event.type) << ","
                 << event.parent.x << "," << event.parent.y << "," << event.parent.theta << ","
                 << event.node.x << "," << event.node.y << "," << event.node.theta << ","
                 << event.cost << "," << event.parent_index << "," << event.node_index << "\n";

        if (event.iteration % 200 == 0) {
            const double percent = 100.0 * static_cast<double>(event.iteration) /
                static_cast<double>(max_iterations);
            std::cout << "Iteration " << event.iteration << " (" << percent << "%)" << std::endl;
        }
    };

    motion_planner::RRTOptions rrt_options;
    rrt_options.step_size = step_size;
    rrt_options.goal_bias = goal_bias;
    rrt_options.max_iterations = max_iterations;
    rrt_options.goal_tolerance = goal_tolerance;
    rrt_options.visualization_sink = visualization_sink;

    motion_planner::RRTStarOptions rrt_star_options;
    rrt_star_options.step_size = step_size;
    rrt_star_options.goal_bias = goal_bias;
    rrt_star_options.max_iterations = max_iterations;
    rrt_star_options.goal_tolerance = goal_tolerance;
    rrt_star_options.neighbor_radius = neighbor_radius;
    rrt_star_options.visualization_sink = visualization_sink;

    // Steering parameters (turning radius, discretization).
    motion_planner::ReedsSheppOptions rs_options;
    rs_options.turning_radius = 1.0;
    rs_options.step_size = 0.2;

    std::cout << "Planning from (" << start.x << ", " << start.y << ", " << start.theta << ") to ("
              << goal.x << ", " << goal.y << ", " << goal.theta << ")." << std::endl;
    std::cout << "Planner: " << (use_rrt_star ? "RRT*" : "RRT") << std::endl;
    std::cout << "Iterations: " << max_iterations
              << ", step size: " << step_size
              << ", goal bias: " << goal_bias << std::endl;
    if (use_rrt_star) {
        std::cout << "RRT* neighbor radius: " << neighbor_radius << std::endl;
    }
    std::cout << "Reeds-Shepp turning radius: " << rs_options.turning_radius
              << ", sampling step: " << rs_options.step_size << std::endl;
    std::cout << "Obstacles: " << obstacles.size() << std::endl;

    // Sampler biased toward goal-aligned headings for Reeds-Shepp feasibility.
    motion_planner::ReedsSheppHeadingSamplerConfig sampler_config;
    sampler_config.min_x = min_x;
    sampler_config.max_x = max_x;
    sampler_config.min_y = min_y;
    sampler_config.max_y = max_y;
    sampler_config.heading_stddev = 0.4;
    sampler_config.reverse_probability = 0.35;

    auto sampler = motion_planner::make_reeds_shepp_heading_sampler(sampler_config, goal);

    // Distance metric used by the planner (SE(2) heuristic).
    auto distance = [&](const State &a, const State &b) {
        return motion_planner::se2_distance(a, b);
    };

    // Collision oracle used along steering paths.
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

    // Reeds-Shepp steering function used by the planner.
    auto steer = [&](const State &from, const State &to) {
        return motion_planner::reeds_shepp_path(from, to, rs_options);
    };

    auto start_time = std::chrono::steady_clock::now();

    // Run the selected planner.
    std::vector<State> path;
    if (use_rrt_star) {
        motion_planner::RRTStar planner(rrt_star_options);
        path = planner.plan(start, goal, sampler, distance, steer, is_state_valid);
    } else {
        motion_planner::RRT planner(rrt_options);
        path = planner.plan(start, goal, sampler, distance, steer, is_state_valid);
    }

    if (path.empty()) {
        std::cout << "No path found.\n";
        return 0;
    }

    // Recompute a dense path along the final sequence of waypoints.
    std::vector<State> full_path;
    full_path.reserve(path.size() * 8);
    for (size_t i = 0; i + 1 < path.size(); ++i) {
        auto segment = steer(path[i], path[i + 1]);
        if (segment.empty()) {
            continue;
        }
        if (!full_path.empty()) {
            segment.erase(segment.begin());
        }
        full_path.insert(full_path.end(), segment.begin(), segment.end());
    }
    if (full_path.empty()) {
        full_path = path;
    }

    std::cout << "Path found. Waypoints: " << path.size()
              << ", discretized states: " << full_path.size() << std::endl;
    std::cout << "Path states: " << full_path.size() << "\n";
    // for (const auto &state : full_path) {
    //     std::cout << state.x << ", " << state.y << ", " << state.theta << "\n";
    //     path_out << state.x << "," << state.y << "," << state.theta << "\n";
    // }

    auto end_time = std::chrono::steady_clock::now();
    const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        end_time - start_time);
    std::cout << "Planning time: " << elapsed_ms.count() << " ms\n";

    return 0;
}
