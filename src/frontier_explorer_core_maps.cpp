/*
Copyright 2026 Mert Güler

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/

#include "frontier_exploration_ros2/frontier_explorer_core.hpp"

#include "frontier_explorer_core_detail.hpp"

namespace frontier_exploration_ros2
{
namespace
{

bool same_costmap_search_input(
  const std::optional<OccupancyGrid2d> & previous,
  const OccupancyGrid2d & next)
{
  if (!previous.has_value()) {
    return false;
  }

  const auto & previous_map = previous->map();
  const auto & next_map = next.map();
  const auto & previous_info = previous_map.info;
  const auto & next_info = next_map.info;

  return (
    previous_info.width == next_info.width &&
    previous_info.height == next_info.height &&
    previous_info.resolution == next_info.resolution &&
    previous_info.origin.position.x == next_info.origin.position.x &&
    previous_info.origin.position.y == next_info.origin.position.y &&
    previous_info.origin.orientation.x == next_info.origin.orientation.x &&
    previous_info.origin.orientation.y == next_info.origin.orientation.y &&
    previous_info.origin.orientation.z == next_info.origin.orientation.z &&
    previous_info.origin.orientation.w == next_info.origin.orientation.w &&
    previous_map.data == next_map.data);
}

}  // namespace

void FrontierExplorerCore::refresh_decision_map()
{
  if (!map.has_value()) {
    return;
  }

  const auto previous_decision_map_msg = decision_map_msg;
  const DecisionMapBuildStatus status = build_decision_map(
    *map,
    decision_map_config(),
    decision_map_workspace);
  decision_map_msg = decision_map_workspace.optimized_map_msg;
  if (
    !decision_map.has_value() ||
    previous_decision_map_msg != decision_map_msg ||
    status.geometry_changed)
  {
    decision_map = OccupancyGrid2d(decision_map_msg);
  }

  if (status.output_changed || !decision_map.has_value()) {
    decision_map_generation += 1;
    decision_map_cache_misses += 1;
    frontier_snapshot.reset();
    decision_map_dirty = false;
    if (debug_outputs_enabled() && decision_map_msg) {
      callbacks.publish_optimized_map(*decision_map_msg);
      callbacks.log_debug(
        "decision_map: miss, raw_generation=" + std::to_string(map_generation) +
        ", decision_generation=" + std::to_string(decision_map_generation) +
        ", optimization_enabled=" +
        std::string(frontier_map_optimization_enabled() ? "true" : "false"));
    }
    return;
  }

  decision_map_cache_hits += 1;
  decision_map_dirty = false;
  if (debug_outputs_enabled()) {
    callbacks.log_debug(
      "decision_map: hit, raw_generation=" + std::to_string(map_generation) +
      ", decision_generation=" + std::to_string(decision_map_generation) +
      ", optimization_enabled=" +
      std::string(frontier_map_optimization_enabled() ? "true" : "false"));
  }
}

void FrontierExplorerCore::ingestRawMapUpdate(const OccupancyGrid2d & map_msg)
{
  map = map_msg;
  map_generation += 1;
  decision_map_dirty = true;
}

bool FrontierExplorerCore::active_frontier_goal_in_progress() const
{
  return goal_in_progress && active_goal_kind == "frontier";
}

void FrontierExplorerCore::handleUrgentRawMapUpdateForActiveGoal()
{
  if (goal_in_progress && active_goal_kind == "frontier") {
    consider_preempt_active_goal("map");
    return;
  }

  if (goal_in_progress && active_goal_kind == "suppressed_return_to_start") {
    consider_cancel_suppressed_return_to_start();
  }
}

void FrontierExplorerCore::processPendingMapUpdate()
{
  const bool had_dirty_map = decision_map_dirty;
  if (decision_map_dirty) {
    refresh_decision_map();
  }

  if (goal_in_progress) {
    if (had_dirty_map && active_goal_kind == "suppressed_return_to_start") {
      consider_cancel_suppressed_return_to_start();
    }
    return;
  }

  if (!had_dirty_map && !awaiting_map_refresh) {
    return;
  }

  try_send_next_goal();
}

void FrontierExplorerCore::occupancyGridCallback(const OccupancyGrid2d & map_msg)
{
  ingestRawMapUpdate(map_msg);
  refresh_decision_map();
  if (goal_in_progress && active_goal_kind == "frontier") {
    // While navigating a frontier, map updates are routed through preemption policy first.
    consider_preempt_active_goal("map");
    return;
  }
  if (goal_in_progress && active_goal_kind == "suppressed_return_to_start") {
    consider_cancel_suppressed_return_to_start();
    return;
  }
  try_send_next_goal();
}

void FrontierExplorerCore::costmapCallback(const OccupancyGrid2d & map_msg)
{
  const bool costmap_changed = !same_costmap_search_input(costmap, map_msg);
  costmap = map_msg;
  if (costmap_changed) {
    costmap_generation += 1;
  }
  if (goal_in_progress && active_goal_kind == "frontier") {
    consider_preempt_active_goal("costmap");
    return;
  }
  if (goal_in_progress && active_goal_kind == "suppressed_return_to_start") {
    consider_cancel_suppressed_return_to_start();
    return;
  }
  if (params.map_processing_rate_hz <= 0.0 || !decision_map_dirty) {
    try_send_next_goal();
  }
}

void FrontierExplorerCore::localCostmapCallback(const OccupancyGrid2d & map_msg)
{
  const bool local_costmap_changed = !same_costmap_search_input(local_costmap, map_msg);
  local_costmap = map_msg;
  if (local_costmap_changed) {
    local_costmap_generation += 1;
  }
  if (goal_in_progress && active_goal_kind == "frontier") {
    consider_preempt_active_goal("costmap");
    return;
  }
  if (goal_in_progress && active_goal_kind == "suppressed_return_to_start") {
    consider_cancel_suppressed_return_to_start();
    return;
  }
  if (params.map_processing_rate_hz <= 0.0 || !decision_map_dirty) {
    try_send_next_goal();
  }
}

}  // namespace frontier_exploration_ros2
