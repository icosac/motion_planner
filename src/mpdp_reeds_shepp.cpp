#include "motion_planner/mpdp_reeds_shepp.hpp"

#include <cmath>
#include <limits>
#include <rs.hh>

namespace motion_planner {
namespace {

constexpr double kEpsilon = 1e-6;

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

} // namespace

std::vector<State> reeds_shepp_path(
    const State &start,
    const State &goal,
    const ReedsSheppOptions &options) {
    if (options.turning_radius <= 0.0 || options.step_size <= 0.0) {
        return {};
    }

    const double kmax = 1.0 / options.turning_radius;

    Configuration2 ci(start.x, start.y, start.theta);
    Configuration2 cf(goal.x, goal.y, goal.theta);
    RS rs(ci, cf, {kmax});
    rs.solve();

    if (!std::isfinite(rs.l()) || rs.getNseg() <= 0) {
        return {};
    }

    const auto lengths = rs.getL();
    const auto curvatures = rs.getK();

    std::vector<State> states;
    states.push_back(start);

    State current = start;
    const double step = std::max(options.step_size, 1e-3);

    for (int i = 0; i < rs.getNseg(); ++i) {
        double remaining = lengths[i];
        while (std::fabs(remaining) > kEpsilon) {
            const double ds = std::copysign(std::min(std::fabs(remaining), step), remaining);
            current = advance_state(current, ds, curvatures[i]);
            states.push_back(current);
            remaining -= ds;
        }
    }

    if (!states.empty()) {
        states.back() = goal;
    }

    return states;
}

} // namespace motion_planner
