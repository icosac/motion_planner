#pragma once

/// \file mpdp_reeds_shepp.hpp
/// \brief Reeds-Shepp steering using the MPDP library.

#include <vector>

#include "motion_planner/types.hpp"

namespace motion_planner {

/// \brief Parameters controlling the MPDP Reeds-Shepp path sampling.
struct ReedsSheppOptions {
    double turning_radius{1.0};
    double step_size{0.1};
};

/// \brief Compute a Reeds-Shepp path between two SE(2) states.
/// \return A sequence of states from start to goal, or empty on failure.
std::vector<State> reeds_shepp_path(
    const State &start,
    const State &goal,
    const ReedsSheppOptions &options);

} // namespace motion_planner
