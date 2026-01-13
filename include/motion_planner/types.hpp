#pragma once

/// \file types.hpp
/// \brief Basic types and helpers for SE(2) planning.

#include <cmath>
#include <vector>

namespace motion_planner {

/// \brief Pi constant used throughout the library.
constexpr double kPi = 3.14159265358979323846;

/// \brief SE(2) state (x, y, heading).
struct State {
    double x{0.0};
    double y{0.0};
    double theta{0.0};
};

/// \brief Tree node used by sampling-based planners.
struct Node {
    State state;
    int parent{-1};
    double cost{0.0};
};

/// \brief Wrap an angle to the [-pi, pi] range.
inline double normalize_angle(double angle) {
    const double two_pi = 2.0 * kPi;
    while (angle > kPi) {
        angle -= two_pi;
    }
    while (angle < -kPi) {
        angle += two_pi;
    }
    return angle;
}

/// \brief Euclidean distance in SE(2), combining position and heading.
inline double se2_distance(const State &a, const State &b) {
    const double dx = a.x - b.x;
    const double dy = a.y - b.y;
    const double dtheta = normalize_angle(a.theta - b.theta);
    return std::sqrt(dx * dx + dy * dy + dtheta * dtheta);
}

} // namespace motion_planner
