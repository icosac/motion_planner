#include "motion_planner/mpdp_reeds_shepp.hpp"

#include <cmath>
#include <iostream>
#include <limits>
#include <RSPredict/RSML.h>

namespace motion_planner {
namespace {

constexpr double kEpsilon = 1e-6;

State advance_state(const State &state, double ds, double curvature) {
    State out = state;
    if (std::fabs(curvature) < 1e-9) {
        // Straight segment update.
        out.x += ds * std::cos(out.theta);
        out.y += ds * std::sin(out.theta);
        return out;
    }

    // Arc segment update with curvature.
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

    std::vector<std::tuple<Configuration2, Configuration2, double>> batch;
    batch.emplace_back(ci, cf, kmax);
    
    // Solve the shortest Reeds-Shepp path with the brute-force RS implementation.
#ifdef USE_ML
    RS rs = gen_rs_from_ml(ci, cf, kmax, "nnwide");
    // std::vector<RS> RSs = gen_rs_from_ml_batch(batch, "nnwide", 5);
#else
    RS rs = RSbruteforce(ci, cf, kmax);
    // std::vector<RS> RSs= RSbruteforceBatch(batch);
#endif

    // RS rs = RSs[0];
    if (!std::isfinite(rs.l()) || rs.getNseg() <= 0) {
        std::cerr << "Reeds-Shepp path computation failed." << std::endl;
        // rs = RSbruteforceBatch(batch)[0];
    }

    const auto lengths = rs.getL();
    const auto curvatures = rs.getK();

    std::vector<State> states;
    states.push_back(start);

    State current = start;
    const double step = std::max(options.step_size, 1e-3);

    // Discretize each segment into fixed-length steps.
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
