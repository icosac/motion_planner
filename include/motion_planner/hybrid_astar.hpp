#pragma once

/// \file hybrid_astar.hpp
/// \brief Hybrid A* planner with Reeds-Shepp goal connection.

#include <functional>
#include <vector>

#include "motion_planner/mpdp_reeds_shepp.hpp"
#include "motion_planner/types.hpp"
#include "motion_planner/visualization.hpp"

namespace motion_planner {

/// \brief Parameters controlling Hybrid A* behavior.
struct HybridAStarOptions {
    double min_x{0.0};
    double max_x{10.0};
    double min_y{0.0};
    double max_y{10.0};
    double resolution{0.5};
    int theta_bins{72};
    double step_size{0.5};
    double turning_radius{1.0};
    bool allow_reverse{true};
    double goal_tolerance{0.5};
    double edge_check_step{0.1};
    int max_expansions{60000};
    bool use_reeds_shepp_goal{true};
    ReedsSheppOptions reeds_shepp{};
    /// \brief Optional callback invoked when nodes are expanded or the goal is reached.
    VisualizationSink visualization_sink{};
};

/// \brief Hybrid A* planner over a grid with discretized headings.
class HybridAStar {
public:
    /// \brief Distance metric between two states.
    using DistanceFn = std::function<double(const State &, const State &)>;
    /// \brief Collision oracle for state validity.
    using ValidityFn = std::function<bool(const State &)>;

    /// \brief Create a Hybrid A* planner with the given options.
    explicit HybridAStar(HybridAStarOptions options);

    /// \brief Planning result with full path samples and lattice nodes.
    struct Result {
        std::vector<State> path;
        std::vector<State> nodes;
    };

    /// \brief Plan a path from start to goal.
    /// \return A sequence of states from start to goal, or empty on failure.
    std::vector<State> plan(
        const State &start,
        const State &goal,
        const DistanceFn &distance,
        const ValidityFn &is_state_valid);

    /// \brief Plan a path and return both samples and lattice nodes.
    Result plan_with_nodes(
        const State &start,
        const State &goal,
        const DistanceFn &distance,
        const ValidityFn &is_state_valid);

private:
    HybridAStarOptions options_;
};

} // namespace motion_planner
