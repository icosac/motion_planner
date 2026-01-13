#pragma once

/// \file rrt.hpp
/// \brief Rapidly-exploring Random Tree (RRT) planner.

#include <functional>
#include <optional>
#include <vector>

#include "motion_planner/types.hpp"
#include "motion_planner/visualization.hpp"

namespace motion_planner {

/// \brief Parameters controlling RRT behavior.
struct RRTOptions {
    double step_size{0.5};
    double goal_bias{0.05};
    int max_iterations{2000};
    double goal_tolerance{0.5};
    /// \brief Optional callback invoked when the tree grows.
    VisualizationSink visualization_sink{};
};

/// \brief RRT planner for SE(2) states with user-defined steering and validity.
class RRT {
public:
    /// \brief Sampling function for random states.
    using SamplerFn = std::function<State()>;
    /// \brief Distance metric between two states.
    using DistanceFn = std::function<double(const State &, const State &)>;
    /// \brief Steering function that returns a path from one state toward another.
    using SteerFn = std::function<std::vector<State>(const State &, const State &)>;
    /// \brief Collision oracle for state validity.
    using ValidityFn = std::function<bool(const State &)>;

    /// \brief Create an RRT planner with the given options.
    explicit RRT(RRTOptions options);

    /// \brief Plan a path from start to goal.
    /// \return A sequence of states from start to goal, or empty on failure.
    std::vector<State> plan(
        const State &start,
        const State &goal,
        const SamplerFn &sampler,
        const DistanceFn &distance,
        const SteerFn &steer,
        const ValidityFn &is_state_valid);

private:
    RRTOptions options_;
};

} // namespace motion_planner
