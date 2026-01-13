#pragma once

/// \file grid_astar.hpp
/// \brief Grid-based A* planner for SE(2) states.

#include <functional>
#include <vector>

#include "motion_planner/types.hpp"
#include "motion_planner/visualization.hpp"

namespace motion_planner {

/// \brief Parameters controlling grid-based A* behavior.
struct GridAStarOptions {
    double resolution{0.5};
    double min_x{0.0};
    double max_x{10.0};
    double min_y{0.0};
    double max_y{10.0};
    double goal_tolerance{0.5};
    bool allow_diagonal{true};
    double edge_check_step{0.0};
    int max_expansions{50000};
    /// \brief Optional callback invoked when a cell is discovered or goal is reached.
    VisualizationSink visualization_sink{};
};

/// \brief Grid-based A* planner over x/y with heading inferred from moves.
/// \note Start/goal are snapped to the nearest grid cell for planning.
class GridAStar {
public:
    /// \brief Distance metric between two states.
    using DistanceFn = std::function<double(const State &, const State &)>;
    /// \brief Collision oracle for state validity.
    using ValidityFn = std::function<bool(const State &)>;

    /// \brief Create a grid planner with the given options.
    explicit GridAStar(GridAStarOptions options);

    /// \brief Plan a path from start to goal.
    /// \return A sequence of states from start to goal, or empty on failure.
    std::vector<State> plan(
        const State &start,
        const State &goal,
        const DistanceFn &distance,
        const ValidityFn &is_state_valid);

private:
    GridAStarOptions options_;
};

} // namespace motion_planner
