#pragma once

/// \file shortcutting.hpp
/// \brief Randomized path shortcutting helpers.

#include <algorithm>
#include <functional>
#include <random>
#include <vector>

#include "motion_planner/types.hpp"

namespace motion_planner {

/// \brief Parameters controlling randomized shortcutting.
struct ShortcutOptions {
    int max_iterations{200};
    double min_improvement{1e-3};
    bool use_seed{false};
    unsigned int seed{0};
};

/// \brief Randomized shortcutting of a path using a steering function.
/// \return The shortened path (or the input if no improvements are found).
inline std::vector<State> randomized_shortcut(
    const std::vector<State> &path,
    const std::function<std::vector<State>(const State &, const State &)> &steer,
    const std::function<double(const State &, const State &)> &distance,
    const std::function<bool(const State &)> &is_state_valid,
    ShortcutOptions options) {
    if (path.size() < 3 || options.max_iterations <= 0) {
        return path;
    }

    auto is_path_valid = [&](const std::vector<State> &segment) {
        for (const auto &state : segment) {
            if (!is_state_valid(state)) {
                return false;
            }
        }
        return true;
    };

    auto path_length = [&](const std::vector<State> &segment) {
        double total = 0.0;
        for (size_t i = 0; i + 1 < segment.size(); ++i) {
            total += distance(segment[i], segment[i + 1]);
        }
        return total;
    };

    std::mt19937 rng(options.use_seed ? options.seed : std::random_device{}());
    std::vector<State> current = path;
    std::uniform_int_distribution<size_t> index_dist(0, current.size() - 1);

    for (int iter = 0; iter < options.max_iterations; ++iter) {
        if (current.size() < 3) {
            break;
        }
        size_t i = index_dist(rng);
        size_t j = index_dist(rng);
        if (i == j) {
            continue;
        }
        if (i > j) {
            std::swap(i, j);
        }
        if (j <= i + 1) {
            continue;
        }

        const auto candidate = steer(current[i], current[j]);
        if (candidate.empty() || !is_path_valid(candidate)) {
            continue;
        }

        double old_cost = 0.0;
        for (size_t k = i; k < j; ++k) {
            old_cost += distance(current[k], current[k + 1]);
        }
        const double new_cost = path_length(candidate);
        if (new_cost + options.min_improvement >= old_cost) {
            continue;
        }

        std::vector<State> updated;
        updated.reserve(current.size() - (j - i - 1) + candidate.size());
        updated.insert(updated.end(), current.begin(), current.begin() + static_cast<long>(i));
        updated.insert(updated.end(), candidate.begin(), candidate.end());
        updated.insert(updated.end(), current.begin() + static_cast<long>(j + 1), current.end());
        current.swap(updated);

        index_dist = std::uniform_int_distribution<size_t>(0, current.size() - 1);
    }

    return current;
}

} // namespace motion_planner
