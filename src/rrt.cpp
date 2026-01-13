#include "motion_planner/rrt.hpp"

#include <algorithm>
#include <random>

namespace motion_planner {
namespace {

State interpolate_state(const State &from, const State &to, double t) {
    State out;
    out.x = from.x + (to.x - from.x) * t;
    out.y = from.y + (to.y - from.y) * t;
    const double dtheta = normalize_angle(to.theta - from.theta);
    out.theta = normalize_angle(from.theta + dtheta * t);
    return out;
}

bool is_path_valid(const std::vector<State> &path, const RRT::ValidityFn &is_state_valid) {
    for (const auto &state : path) {
        if (!is_state_valid(state)) {
            return false;
        }
    }
    return true;
}

std::vector<State> truncate_path(
    const State &from,
    const std::vector<State> &path,
    double max_distance,
    const RRT::DistanceFn &distance,
    const RRT::ValidityFn &is_state_valid) {
    std::vector<State> truncated;
    State current = from;
    double traveled = 0.0;

    for (const auto &state : path) {
        const double seg = distance(current, state);
        if (seg <= 0.0) {
            continue;
        }

        if (traveled + seg > max_distance) {
            const double remaining = max_distance - traveled;
            if (remaining <= 0.0) {
                break;
            }
            const double ratio = remaining / seg;
            State interp = interpolate_state(current, state, ratio);
            if (!is_state_valid(interp)) {
                break;
            }
            truncated.push_back(interp);
            break;
        }

        if (!is_state_valid(state)) {
            break;
        }

        truncated.push_back(state);
        traveled += seg;
        current = state;
    }

    return truncated;
}

std::vector<State> reconstruct_path(const std::vector<Node> &nodes, int goal_index) {
    std::vector<State> path;
    int current = goal_index;
    while (current >= 0) {
        path.push_back(nodes[current].state);
        current = nodes[current].parent;
    }
    std::reverse(path.begin(), path.end());
    return path;
}

} // namespace

RRT::RRT(RRTOptions options) : options_(options) {}

std::vector<State> RRT::plan(
    const State &start,
    const State &goal,
    const SamplerFn &sampler,
    const DistanceFn &distance,
    const SteerFn &steer,
    const ValidityFn &is_state_valid) {
    std::vector<Node> nodes;
    nodes.push_back(Node{start, -1, 0.0});

    std::mt19937 rng(std::random_device{}());
    std::uniform_real_distribution<double> uni(0.0, 1.0);

    for (int iter = 0; iter < options_.max_iterations; ++iter) {
        // Goal bias mixes random exploration with occasional goal sampling.
        const bool use_goal = uni(rng) < options_.goal_bias;
        const State target = use_goal ? goal : sampler();

        int nearest_index = 0;
        double best_distance = distance(nodes[0].state, target);
        for (int i = 1; i < static_cast<int>(nodes.size()); ++i) {
            const double d = distance(nodes[i].state, target);
            if (d < best_distance) {
                best_distance = d;
                nearest_index = i;
            }
        }

        // Steer toward the target and truncate to a fixed step size.
        const auto full_path = steer(nodes[nearest_index].state, target);
        if (full_path.empty()) {
            continue;
        }

        const auto truncated = truncate_path(
            nodes[nearest_index].state,
            full_path,
            options_.step_size,
            distance,
            is_state_valid);

        if (truncated.empty()) {
            continue;
        }

        const State new_state = truncated.back();
        const double new_cost = nodes[nearest_index].cost + distance(nodes[nearest_index].state, new_state);
        nodes.push_back(Node{new_state, nearest_index, new_cost});
        const int new_index = static_cast<int>(nodes.size()) - 1;

        if (options_.visualization_sink) {
            VisualizationEvent event;
            event.type = VisualizationEventType::Add;
            event.iteration = iter;
            event.node_index = new_index;
            event.parent_index = nearest_index;
            event.node = new_state;
            event.parent = nodes[nearest_index].state;
            event.cost = new_cost;
            options_.visualization_sink(event);
        }

        // If close enough, attempt to connect directly to the goal.
        if (distance(new_state, goal) <= options_.goal_tolerance) {
            const auto goal_path = steer(new_state, goal);
            if (!goal_path.empty() && is_path_valid(goal_path, is_state_valid)) {
                const double goal_cost = nodes[new_index].cost + distance(new_state, goal);
                nodes.push_back(Node{goal, new_index, goal_cost});
                if (options_.visualization_sink) {
                    VisualizationEvent event;
                    event.type = VisualizationEventType::Goal;
                    event.iteration = iter;
                    event.node_index = static_cast<int>(nodes.size()) - 1;
                    event.parent_index = new_index;
                    event.node = goal;
                    event.parent = nodes[new_index].state;
                    event.cost = goal_cost;
                    options_.visualization_sink(event);
                }
                return reconstruct_path(nodes, static_cast<int>(nodes.size()) - 1);
            }
        }
    }

    return {};
}

} // namespace motion_planner
