#include "motion_planner/hybrid_astar.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <queue>
#include <utility>

namespace motion_planner {
namespace {

struct NodeInfo {
    State state{};
    double g{std::numeric_limits<double>::infinity()};
    int parent{-1};
    double curvature{0.0};
    double length{0.0};
    bool closed{false};
};

struct QueueEntry {
    int index{0};
    double f{0.0};
    double g{0.0};
};

struct QueueCompare {
    bool operator()(const QueueEntry &lhs, const QueueEntry &rhs) const {
        return lhs.f > rhs.f;
    }
};

State advance_state(const State &state, double ds, double curvature) {
    State out = state;
    if (std::fabs(curvature) < 1e-9) {
        out.x += ds * std::cos(out.theta);
        out.y += ds * std::sin(out.theta);
        return out;
    }

    const double dtheta = curvature * ds;
    out.x += (std::sin(out.theta + dtheta) - std::sin(out.theta)) / curvature;
    out.y += (-std::cos(out.theta + dtheta) + std::cos(out.theta)) / curvature;
    out.theta = normalize_angle(out.theta + dtheta);
    return out;
}

int flatten_index(int ix, int iy, int itheta, int width, int height, int theta_bins) {
    return (iy * width + ix) * theta_bins + itheta;
}

bool state_to_index(
    const State &state,
    const HybridAStarOptions &options,
    int width,
    int height,
    int theta_bins,
    int &ix,
    int &iy,
    int &itheta) {
    const double eps = 1e-9;
    if (state.x < options.min_x - eps || state.x > options.max_x + eps) {
        return false;
    }
    if (state.y < options.min_y - eps || state.y > options.max_y + eps) {
        return false;
    }

    ix = static_cast<int>(std::round((state.x - options.min_x) / options.resolution));
    iy = static_cast<int>(std::round((state.y - options.min_y) / options.resolution));
    ix = std::clamp(ix, 0, width - 1);
    iy = std::clamp(iy, 0, height - 1);

    const double two_pi = 2.0 * kPi;
    double theta = normalize_angle(state.theta);
    if (theta < 0.0) {
        theta += two_pi;
    }
    const double bin_size = two_pi / static_cast<double>(theta_bins);
    itheta = static_cast<int>(std::floor(theta / bin_size));
    if (itheta >= theta_bins) {
        itheta = theta_bins - 1;
    }
    return true;
}

bool simulate_primitive(
    const State &start,
    double curvature,
    double length,
    double check_step,
    const HybridAStar::ValidityFn &is_state_valid,
    State &out) {
    const double abs_length = std::fabs(length);
    if (abs_length <= 1e-9) {
        out = start;
        return true;
    }
    int steps = 1;
    if (check_step > 0.0) {
        steps = std::max(1, static_cast<int>(std::ceil(abs_length / check_step)));
    }
    const double ds = length / static_cast<double>(steps);
    State current = start;
    for (int i = 0; i < steps; ++i) {
        current = advance_state(current, ds, curvature);
        if (!is_state_valid(current)) {
            return false;
        }
    }
    out = current;
    return true;
}

void append_primitive_samples(
    const State &start,
    double curvature,
    double length,
    double sample_step,
    std::vector<State> &out) {
    const double abs_length = std::fabs(length);
    if (abs_length <= 1e-9) {
        return;
    }
    int steps = 1;
    if (sample_step > 0.0) {
        steps = std::max(1, static_cast<int>(std::ceil(abs_length / sample_step)));
    }
    const double ds = length / static_cast<double>(steps);
    State current = start;
    for (int i = 0; i < steps; ++i) {
        current = advance_state(current, ds, curvature);
        out.push_back(current);
    }
}

bool is_path_valid(const std::vector<State> &path, const HybridAStar::ValidityFn &is_state_valid) {
    for (const auto &state : path) {
        if (!is_state_valid(state)) {
            return false;
        }
    }
    return true;
}

std::vector<State> reconstruct_path(
    const std::vector<NodeInfo> &nodes,
    int goal_index,
    double sample_step,
    std::vector<State> *node_path) {
    std::vector<int> indices;
    int current = goal_index;
    while (current >= 0) {
        indices.push_back(current);
        current = nodes[current].parent;
    }
    std::reverse(indices.begin(), indices.end());

    std::vector<State> path;
    if (indices.empty()) {
        return path;
    }
    if (node_path) {
        node_path->clear();
        node_path->reserve(indices.size());
        for (int idx : indices) {
            node_path->push_back(nodes[idx].state);
        }
    }
    path.push_back(nodes[indices.front()].state);

    for (size_t i = 1; i < indices.size(); ++i) {
        const int idx = indices[i];
        const int parent = nodes[idx].parent;
        append_primitive_samples(
            nodes[parent].state,
            nodes[idx].curvature,
            nodes[idx].length,
            sample_step,
            path);
    }
    return path;
}

} // namespace

HybridAStar::HybridAStar(HybridAStarOptions options) : options_(options) {}

std::vector<State> HybridAStar::plan(
    const State &start,
    const State &goal,
    const DistanceFn &distance,
    const ValidityFn &is_state_valid) {
    return plan_with_nodes(start, goal, distance, is_state_valid).path;
}

HybridAStar::Result HybridAStar::plan_with_nodes(
    const State &start,
    const State &goal,
    const DistanceFn &distance,
    const ValidityFn &is_state_valid) {
    Result result;
    if (options_.resolution <= 0.0 || options_.theta_bins <= 0) {
        return result;
    }
    if (options_.step_size <= 0.0 || options_.turning_radius <= 0.0) {
        return result;
    }
    if (options_.max_x < options_.min_x || options_.max_y < options_.min_y) {
        return result;
    }

    const double span_x = options_.max_x - options_.min_x;
    const double span_y = options_.max_y - options_.min_y;
    const int width = static_cast<int>(std::floor(span_x / options_.resolution)) + 1;
    const int height = static_cast<int>(std::floor(span_y / options_.resolution)) + 1;
    if (width <= 0 || height <= 0) {
        return result;
    }

    if (!is_state_valid(start) || !is_state_valid(goal)) {
        return result;
    }

    int start_x = 0;
    int start_y = 0;
    int start_theta = 0;
    if (!state_to_index(start, options_, width, height, options_.theta_bins, start_x, start_y, start_theta)) {
        return result;
    }

    const int total = width * height * options_.theta_bins;
    std::vector<NodeInfo> nodes(static_cast<size_t>(total));

    const int start_index = flatten_index(start_x, start_y, start_theta, width, height, options_.theta_bins);
    nodes[start_index].state = start;
    nodes[start_index].g = 0.0;
    nodes[start_index].parent = -1;
    nodes[start_index].curvature = 0.0;
    nodes[start_index].length = 0.0;

    std::priority_queue<QueueEntry, std::vector<QueueEntry>, QueueCompare> open;
    open.push(QueueEntry{start_index, distance(start, goal), 0.0});

    const double kmax = 1.0 / options_.turning_radius;
    const std::array<double, 3> curvatures = {{-kmax, 0.0, kmax}};
    const std::array<int, 2> directions = {{1, -1}};

    const double sample_step = (options_.edge_check_step > 0.0)
        ? options_.edge_check_step
        : std::max(0.2 * options_.step_size, 1e-3);

    int expansions = 0;

    while (!open.empty() && expansions < options_.max_expansions) {
        const QueueEntry current_entry = open.top();
        open.pop();

        if (nodes[current_entry.index].closed) {
            continue;
        }
        if (current_entry.g > nodes[current_entry.index].g + 1e-9) {
            continue;
        }

        const int iter = expansions;
        ++expansions;
        nodes[current_entry.index].closed = true;
        const State current_state = nodes[current_entry.index].state;

        if (distance(current_state, goal) <= options_.goal_tolerance) {
            if (options_.use_reeds_shepp_goal) {
                ReedsSheppOptions rs_options = options_.reeds_shepp;
                if (rs_options.turning_radius <= 0.0) {
                    rs_options.turning_radius = options_.turning_radius;
                }
                if (rs_options.step_size <= 0.0) {
                    rs_options.step_size = std::max(sample_step, 0.05);
                }
                auto rs_path = reeds_shepp_path(current_state, goal, rs_options);
                if (!rs_path.empty() && is_path_valid(rs_path, is_state_valid)) {
                    auto path = reconstruct_path(
                        nodes,
                        current_entry.index,
                        sample_step,
                        &result.nodes);
                    if (!path.empty()) {
                        rs_path.erase(rs_path.begin());
                    }
                    path.insert(path.end(), rs_path.begin(), rs_path.end());
                    if (options_.visualization_sink) {
                        VisualizationEvent event;
                        event.type = VisualizationEventType::Goal;
                        event.iteration = iter;
                        event.node_index = -1;
                        event.parent_index = current_entry.index;
                        event.node = goal;
                        event.parent = current_state;
                        event.cost = nodes[current_entry.index].g + distance(current_state, goal);
                        options_.visualization_sink(event);
                    }
                    result.path = std::move(path);
                    return result;
                }
            } else {
                auto path = reconstruct_path(
                    nodes,
                    current_entry.index,
                    sample_step,
                    &result.nodes);
                if (!path.empty()) {
                    path.push_back(goal);
                }
                result.path = std::move(path);
                return result;
            }
        }

        for (double curvature : curvatures) {
            for (int dir : directions) {
                if (dir < 0 && !options_.allow_reverse) {
                    continue;
                }
                const double length = options_.step_size * static_cast<double>(dir);
                State next_state;
                if (!simulate_primitive(
                        current_state,
                        curvature,
                        length,
                        options_.edge_check_step,
                        is_state_valid,
                        next_state)) {
                    continue;
                }

                int next_x = 0;
                int next_y = 0;
                int next_theta = 0;
                if (!state_to_index(
                        next_state,
                        options_,
                        width,
                        height,
                        options_.theta_bins,
                        next_x,
                        next_y,
                        next_theta)) {
                    continue;
                }

                const int next_index = flatten_index(
                    next_x, next_y, next_theta, width, height, options_.theta_bins);
                if (nodes[next_index].closed) {
                    continue;
                }

                const double step_cost = std::fabs(length);
                const double tentative_g = nodes[current_entry.index].g + step_cost;
                if (tentative_g + 1e-9 >= nodes[next_index].g) {
                    continue;
                }

                nodes[next_index].state = next_state;
                nodes[next_index].g = tentative_g;
                nodes[next_index].parent = current_entry.index;
                nodes[next_index].curvature = curvature;
                nodes[next_index].length = length;

                const double h = distance(next_state, goal);
                open.push(QueueEntry{next_index, tentative_g + h, tentative_g});

                if (options_.visualization_sink) {
                    VisualizationEvent event;
                    event.type = VisualizationEventType::Add;
                    event.iteration = iter;
                    event.node_index = next_index;
                    event.parent_index = current_entry.index;
                    event.node = next_state;
                    event.parent = current_state;
                    event.cost = tentative_g;
                    options_.visualization_sink(event);
                }
            }
        }
    }

    return result;
}

} // namespace motion_planner
