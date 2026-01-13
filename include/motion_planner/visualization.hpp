#pragma once

/// \file visualization.hpp
/// \brief Visualization callbacks for planner tree growth.

#include <functional>

#include "motion_planner/types.hpp"

namespace motion_planner {

/// \brief Type of visualization event emitted by planners.
enum class VisualizationEventType {
    Add,
    Rewire,
    Goal
};

/// \brief Information about a tree update for visualization.
struct VisualizationEvent {
    VisualizationEventType type{VisualizationEventType::Add};
    int iteration{0};
    int node_index{-1};
    int parent_index{-1};
    State node;
    State parent;
    double cost{0.0};
};

/// \brief Callback signature for visualization events.
using VisualizationSink = std::function<void(const VisualizationEvent &)>;

} // namespace motion_planner
