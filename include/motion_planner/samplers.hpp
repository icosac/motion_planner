#pragma once

/// \file samplers.hpp
/// \brief Helper samplers for SE(2) planning.

#include <functional>
#include <random>

#include "motion_planner/types.hpp"

namespace motion_planner {

/// \brief Configuration for uniform SE(2) sampling.
struct UniformSamplerConfig {
    double min_x{0.0};
    double max_x{1.0};
    double min_y{0.0};
    double max_y{1.0};
    double min_theta{-kPi};
    double max_theta{kPi};
};

/// \brief Configuration for SE(2) sampling with heading bias toward a target.
struct GoalHeadingSamplerConfig {
    double min_x{0.0};
    double max_x{1.0};
    double min_y{0.0};
    double max_y{1.0};
    double heading_stddev{0.5};
};

/// \brief Configuration for sampling with Reeds-Shepp-feasible headings.
struct ReedsSheppHeadingSamplerConfig {
    double min_x{0.0};
    double max_x{1.0};
    double min_y{0.0};
    double max_y{1.0};
    double heading_stddev{0.4};
    double reverse_probability{0.35};
};

/// \brief Uniform sampling over x, y, theta.
/// \note The returned sampler owns its own RNG state.
inline std::function<State()> make_uniform_sampler(UniformSamplerConfig config) {
    std::mt19937 rng(std::random_device{}());
    std::uniform_real_distribution<double> x_dist(config.min_x, config.max_x);
    std::uniform_real_distribution<double> y_dist(config.min_y, config.max_y);
    std::uniform_real_distribution<double> theta_dist(config.min_theta, config.max_theta);

    return [rng, x_dist, y_dist, theta_dist]() mutable {
        return State{x_dist(rng), y_dist(rng), theta_dist(rng)};
    };
}

/// \brief Sample x,y uniformly and bias heading toward the line from sample to goal.
/// \note The returned sampler owns its own RNG state.
inline std::function<State()> make_goal_heading_sampler(
    GoalHeadingSamplerConfig config,
    State goal) {
    std::mt19937 rng(std::random_device{}());
    std::uniform_real_distribution<double> x_dist(config.min_x, config.max_x);
    std::uniform_real_distribution<double> y_dist(config.min_y, config.max_y);
    std::normal_distribution<double> heading_dist(0.0, config.heading_stddev);

    return [rng, x_dist, y_dist, heading_dist, goal]() mutable {
        const double x = x_dist(rng);
        const double y = y_dist(rng);
        const double base = std::atan2(goal.y - y, goal.x - x);
        const double theta = normalize_angle(base + heading_dist(rng));
        return State{x, y, theta};
    };
}

/// \brief Sample x,y uniformly and bias heading toward goal forward or reverse.
/// \note The returned sampler owns its own RNG state.
inline std::function<State()> make_reeds_shepp_heading_sampler(
    ReedsSheppHeadingSamplerConfig config,
    State goal) {
    std::mt19937 rng(std::random_device{}());
    std::uniform_real_distribution<double> x_dist(config.min_x, config.max_x);
    std::uniform_real_distribution<double> y_dist(config.min_y, config.max_y);
    std::normal_distribution<double> heading_dist(0.0, config.heading_stddev);
    std::bernoulli_distribution reverse_dist(config.reverse_probability);

    return [rng, x_dist, y_dist, heading_dist, reverse_dist, goal]() mutable {
        const double x = x_dist(rng);
        const double y = y_dist(rng);
        double base = std::atan2(goal.y - y, goal.x - x);
        if (reverse_dist(rng)) {
            base = normalize_angle(base + kPi);
        }
        const double theta = normalize_angle(base + heading_dist(rng));
        return State{x, y, theta};
    };
}

} // namespace motion_planner
