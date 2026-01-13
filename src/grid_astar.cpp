#include "motion_planner/grid_astar.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <queue>
#include <utility>

namespace motion_planner {
namespace {

struct CellInfo {
    double g{std::numeric_limits<double>::infinity()};
    int parent{-1};
    double theta{0.0};
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

State interpolate_state(const State &from, const State &to, double t) {
    State out;
    out.x = from.x + (to.x - from.x) * t;
    out.y = from.y + (to.y - from.y) * t;
    const double dtheta = normalize_angle(to.theta - from.theta);
    out.theta = normalize_angle(from.theta + dtheta * t);
    return out;
}

bool is_edge_valid(
    const State &from,
    const State &to,
    double step,
    const GridAStar::ValidityFn &is_state_valid) {
    // Optional edge sampling to avoid "tunneling" through obstacles.
    if (step <= 0.0) {
        return true;
    }
    const double dx = to.x - from.x;
    const double dy = to.y - from.y;
    const double dist = std::hypot(dx, dy);
    if (dist <= 0.0) {
        return true;
    }
    const int steps = static_cast<int>(std::floor(dist / step));
    for (int i = 1; i <= steps; ++i) {
        const double t = (static_cast<double>(i) * step) / dist;
        const State interp = interpolate_state(from, to, t);
        if (!is_state_valid(interp)) {
            return false;
        }
    }
    return true;
}

int flatten_index(int ix, int iy, int width) {
    return iy * width + ix;
}

State state_from_index(int ix, int iy, double theta, const GridAStarOptions &options) {
    State state;
    state.x = options.min_x + static_cast<double>(ix) * options.resolution;
    state.y = options.min_y + static_cast<double>(iy) * options.resolution;
    state.theta = normalize_angle(theta);
    return state;
}

bool position_to_index(
    const GridAStarOptions &options,
    int width,
    int height,
    double x,
    double y,
    int &ix,
    int &iy) {
    // Snap to the closest valid grid index within bounds.
    const double eps = 1e-9;
    if (x < options.min_x - eps || x > options.max_x + eps) {
        return false;
    }
    if (y < options.min_y - eps || y > options.max_y + eps) {
        return false;
    }
    ix = static_cast<int>(std::round((x - options.min_x) / options.resolution));
    iy = static_cast<int>(std::round((y - options.min_y) / options.resolution));
    ix = std::clamp(ix, 0, width - 1);
    iy = std::clamp(iy, 0, height - 1);
    return true;
}

std::vector<State> reconstruct_path(
    const std::vector<CellInfo> &cells,
    int width,
    int goal_index,
    const GridAStarOptions &options) {
    // Walk parent links back to the start and reverse.
    std::vector<int> indices;
    int current = goal_index;
    while (current >= 0) {
        indices.push_back(current);
        current = cells[current].parent;
    }
    std::reverse(indices.begin(), indices.end());

    std::vector<State> path;
    path.reserve(indices.size());
    for (int idx : indices) {
        const int ix = idx % width;
        const int iy = idx / width;
        path.push_back(state_from_index(ix, iy, cells[idx].theta, options));
    }
    return path;
}

} // namespace

GridAStar::GridAStar(GridAStarOptions options) : options_(options) {}

std::vector<State> GridAStar::plan(
    const State &start,
    const State &goal,
    const DistanceFn &distance,
    const ValidityFn &is_state_valid) {
    if (options_.resolution <= 0.0 || options_.max_expansions <= 0) {
        return {};
    }
    if (options_.max_x < options_.min_x || options_.max_y < options_.min_y) {
        return {};
    }

    const double span_x = options_.max_x - options_.min_x;
    const double span_y = options_.max_y - options_.min_y;
    // Build a fixed-resolution grid that includes both min and max bounds.
    const int width = static_cast<int>(std::floor(span_x / options_.resolution)) + 1;
    const int height = static_cast<int>(std::floor(span_y / options_.resolution)) + 1;
    if (width <= 0 || height <= 0) {
        return {};
    }

    int start_x = 0;
    int start_y = 0;
    if (!position_to_index(options_, width, height, start.x, start.y, start_x, start_y)) {
        return {};
    }

    const State snapped_start = state_from_index(start_x, start_y, start.theta, options_);
    if (!is_state_valid(snapped_start) || !is_state_valid(goal)) {
        return {};
    }

    std::vector<CellInfo> cells(static_cast<size_t>(width) * static_cast<size_t>(height));
    const int start_index = flatten_index(start_x, start_y, width);
    cells[start_index].g = 0.0;
    cells[start_index].parent = -1;
    cells[start_index].theta = normalize_angle(start.theta);

    std::priority_queue<QueueEntry, std::vector<QueueEntry>, QueueCompare> open;
    // Seed with the start node and its heuristic cost to the goal.
    open.push(QueueEntry{start_index, distance(snapped_start, goal), 0.0});

    const std::array<std::pair<int, int>, 4> offsets4 = {{
        {1, 0},
        {-1, 0},
        {0, 1},
        {0, -1},
    }};
    const std::array<std::pair<int, int>, 8> offsets8 = {{
        {1, 0},
        {-1, 0},
        {0, 1},
        {0, -1},
        {1, 1},
        {1, -1},
        {-1, 1},
        {-1, -1},
    }};

    // Safety cap to avoid expanding the entire grid unintentionally.
    int expansions = 0;

    while (!open.empty() && expansions < options_.max_expansions) {
        const QueueEntry current_entry = open.top();
        open.pop();

        // Skip stale queue entries or already-expanded cells.
        if (cells[current_entry.index].closed) {
            continue;
        }
        if (current_entry.g > cells[current_entry.index].g + 1e-9) {
            continue;
        }

        const int iter = expansions;
        ++expansions;
        cells[current_entry.index].closed = true;

        const int ix = current_entry.index % width;
        const int iy = current_entry.index / width;
        const State current_state = state_from_index(ix, iy, cells[current_entry.index].theta, options_);

        // Stop when close enough to the goal and a direct edge is valid.
        if (distance(current_state, goal) <= options_.goal_tolerance) {
            if (is_edge_valid(current_state, goal, options_.edge_check_step, is_state_valid)) {
                auto path = reconstruct_path(cells, width, current_entry.index, options_);
                if (path.empty() || distance(path.back(), goal) > 1e-9) {
                    path.push_back(goal);
                }
                if (options_.visualization_sink) {
                    VisualizationEvent event;
                    event.type = VisualizationEventType::Goal;
                    event.iteration = iter;
                    event.node_index = -1;
                    event.parent_index = current_entry.index;
                    event.node = goal;
                    event.parent = current_state;
                    event.cost = cells[current_entry.index].g + distance(current_state, goal);
                    options_.visualization_sink(event);
                }
                return path;
            }
        }

        // Use 4-connected or 8-connected neighbor expansion.
        if (options_.allow_diagonal) {
            for (const auto &offset : offsets8) {
                const int nx = ix + offset.first;
                const int ny = iy + offset.second;
                if (nx < 0 || nx >= width || ny < 0 || ny >= height) {
                    continue;
                }
                const int neighbor_index = flatten_index(nx, ny, width);
                if (cells[neighbor_index].closed) {
                    continue;
                }

                State neighbor_state = state_from_index(nx, ny, 0.0, options_);
                // Infer heading from the grid move for visualization and distance heuristics.
                const double heading = std::atan2(
                    neighbor_state.y - current_state.y,
                    neighbor_state.x - current_state.x);
                neighbor_state.theta = normalize_angle(heading);

                if (!is_state_valid(neighbor_state)) {
                    continue;
                }
                if (!is_edge_valid(current_state, neighbor_state, options_.edge_check_step, is_state_valid)) {
                    continue;
                }

                const double tentative_g = cells[current_entry.index].g + distance(current_state, neighbor_state);
                if (tentative_g + 1e-9 >= cells[neighbor_index].g) {
                    continue;
                }

                // Found a better path to this neighbor.
                cells[neighbor_index].g = tentative_g;
                cells[neighbor_index].parent = current_entry.index;
                cells[neighbor_index].theta = neighbor_state.theta;

                const double h = distance(neighbor_state, goal);
                open.push(QueueEntry{neighbor_index, tentative_g + h, tentative_g});

                if (options_.visualization_sink) {
                    VisualizationEvent event;
                    event.type = VisualizationEventType::Add;
                    event.iteration = iter;
                    event.node_index = neighbor_index;
                    event.parent_index = current_entry.index;
                    event.node = neighbor_state;
                    event.parent = current_state;
                    event.cost = tentative_g;
                    options_.visualization_sink(event);
                }
            }
        } else {
            for (const auto &offset : offsets4) {
                const int nx = ix + offset.first;
                const int ny = iy + offset.second;
                if (nx < 0 || nx >= width || ny < 0 || ny >= height) {
                    continue;
                }
                const int neighbor_index = flatten_index(nx, ny, width);
                if (cells[neighbor_index].closed) {
                    continue;
                }

                State neighbor_state = state_from_index(nx, ny, 0.0, options_);
                // Infer heading from the grid move for visualization and distance heuristics.
                const double heading = std::atan2(
                    neighbor_state.y - current_state.y,
                    neighbor_state.x - current_state.x);
                neighbor_state.theta = normalize_angle(heading);

                if (!is_state_valid(neighbor_state)) {
                    continue;
                }
                if (!is_edge_valid(current_state, neighbor_state, options_.edge_check_step, is_state_valid)) {
                    continue;
                }

                const double tentative_g = cells[current_entry.index].g + distance(current_state, neighbor_state);
                if (tentative_g + 1e-9 >= cells[neighbor_index].g) {
                    continue;
                }

                // Found a better path to this neighbor.
                cells[neighbor_index].g = tentative_g;
                cells[neighbor_index].parent = current_entry.index;
                cells[neighbor_index].theta = neighbor_state.theta;

                const double h = distance(neighbor_state, goal);
                open.push(QueueEntry{neighbor_index, tentative_g + h, tentative_g});

                if (options_.visualization_sink) {
                    VisualizationEvent event;
                    event.type = VisualizationEventType::Add;
                    event.iteration = iter;
                    event.node_index = neighbor_index;
                    event.parent_index = current_entry.index;
                    event.node = neighbor_state;
                    event.parent = current_state;
                    event.cost = tentative_g;
                    options_.visualization_sink(event);
                }
            }
        }
    }

    return {};
}

} // namespace motion_planner
