#include "astar_planner/astar_planner.hpp"

#include <chrono>

#include "occ_map/cost_values.hpp"

using namespace std::chrono_literals;
using namespace std::chrono; // NOLINT
using nav2_util::declare_parameter_if_not_declared;

namespace astar_planner
{

AStarPlanner::AStarPlanner()
    : tf_(nullptr), occ_map_(nullptr)
{
}

AStarPlanner::~AStarPlanner()
{
  RCLCPP_INFO(
      logger_, "Destroying plugin %s of type AStarPlanner",
      name_.c_str());
}

void
AStarPlanner::configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<occ_map::OccMap> occ_map)
{
  tf_ = tf;
  name_ = name;

  occ_map_ = occ_map;

  node_ = parent;
  auto node = parent.lock();
  clock_ = node->get_clock();
  logger_ = node->get_logger();

  RCLCPP_INFO(
      logger_, "Configuring plugin %s of type AStarPlanner",
      name_.c_str());

  // Initialize parameters
  // Declare this plugin's parameters
  declare_parameter_if_not_declared(node, name + ".print_runtime", rclcpp::ParameterValue(false));
  node->get_parameter(name + ".print_runtime", print_runtime_);
  declare_parameter_if_not_declared(node, name + ".tolerance", rclcpp::ParameterValue(0.5));
  node->get_parameter(name + ".tolerance", tolerance_);
  declare_parameter_if_not_declared(node, name + ".max_iterations", rclcpp::ParameterValue(99999));
  node->get_parameter(name + ".max_iterations", max_iterations_);
  declare_parameter_if_not_declared(node, name + ".tie_breaker", rclcpp::ParameterValue(1.001));
  node->get_parameter(name + ".tie_breaker", tie_breaker_);
  declare_parameter_if_not_declared(node, name + ".allow_unknown", rclcpp::ParameterValue(true));
  node->get_parameter(name + ".allow_unknown", allow_unknown_);
  declare_parameter_if_not_declared(node, name + ".cost_function_type", rclcpp::ParameterValue(2));
  node->get_parameter(name + ".cost_function_type", cost_function_type_);

  // Create a planner based on the new costmap size
  planner_ = std::make_unique<AStar>();
}

void
AStarPlanner::activate()
{
  RCLCPP_INFO(
      logger_, "Activating plugin %s of type AStarPlanner",
      name_.c_str());
  // Add callback for dynamic parameters
  auto node = node_.lock();
  // dyn_params_handler_ = node->add_on_set_parameters_callback(
  //     std::bind(&AStarPlanner::dynamicParametersCallback, this, _1));
}

void
AStarPlanner::deactivate()
{
  RCLCPP_INFO(
      logger_, "Deactivating plugin %s of type AStarPlanner",
      name_.c_str());
  auto node = node_.lock();
  if (dyn_params_handler_ && node)
  {
    node->remove_on_set_parameters_callback(dyn_params_handler_.get());
  }
  dyn_params_handler_.reset();
}

void
AStarPlanner::cleanup()
{
  RCLCPP_INFO(
      logger_, "Cleaning up plugin %s of type AStarPlanner",
      name_.c_str());
  planner_.reset();
}

nav_msgs::msg::Path AStarPlanner::createPlan(
    const Eigen::Vector3d &start,
    const Eigen::Vector3d &goal,
    std::function<bool()> cancel_checker)
{

  // Search takes place in index space. So we first convert 3d real world positions into indices
  if (!occ_map_->inGlobalMap(start))
  {
    throw gestelt_core::StartOutsideMapBounds(
        "Start Coordinates of(" + std::to_string(start(0)) + ", " +
        std::to_string(start(1)) + "," + std::to_string(start(2)) + ") was outside bounds");
  }

  if (!occ_map_->inGlobalMap(goal))
  {
    throw gestelt_core::GoalOutsideMapBounds(
      "Goal Coordinates of(" + std::to_string(goal(0)) + ", " +
      std::to_string(goal(1)) + ", " + std::to_string(goal(2)) + ") was outside bounds");

  }

  if (tolerance_ == 0 && occ_map_->getCost(goal) == occ_map::LETHAL_OBSTACLE)
  {
    throw gestelt_core::GoalOccupied(
        "Goal Coordinates of(" + std::to_string(goal(0)) + ", " +
        std::to_string(goal(1)) + ", " + std::to_string(goal(2)) + ") was in lethal cost");
  }

  nav_msgs::msg::Path path;

  if (!makePlan(start, goal, tolerance_, cancel_checker, path))
  {
    throw gestelt_core::NoValidPathCouldBeFound(
        "Failed to create plan with tolerance of: " + std::to_string(tolerance_));
  }

  return path;
}

bool
AStarPlanner::makePlan(
    const Eigen::Vector3d &start,
    const Eigen::Vector3d &goal, double tolerance,
    std::function<bool()> cancel_checker,
    nav_msgs::msg::Path &plan)
{
  // clear the plan, just in case
  plan.poses.clear();

  plan.header.stamp = clock_->now();
  plan.header.frame_id = occ_map_->getGlobalFrameID();

  Eigen::Vector3d map_start, map_goal;

  occ_map_->worldToMap(start, map_start);
  occ_map_->worldToMap(goal, map_goal);

  RCLCPP_INFO(
    logger_, "AStarPlanner::makePlan in map_frame from (%.2f, %.2f, %.2f) to "
    "(%.2f, %.2f, %.2f).", 
    map_start(0), map_start(1), map_start(2),
    map_goal(0), map_goal(1), map_goal(2));

  // clear the starting cell within the costmap because we know it can't be an obstacle
  // clearRobotCell(map_start);

  std::unique_lock<occ_map::OccMap::mutex_t> lock(*(occ_map_->getMutex()));

  planner_->setCostmap(occ_map_);

  planner_->setStart(map_goal);
  planner_->setGoal(map_start);

  int path_len = planner_->computePath(max_iterations_, cancel_checker);
  if (path_len == 0) {
    return false;
  }

  lock.unlock();

  RCLCPP_INFO(
    logger_, "Size of path: %d", path_len);

  // Obtain planned path in map frame
  auto map_planned_path = planner_->getPath();

  for (size_t i = 0; i < map_planned_path.size(); i++) {
    Eigen::Vector3d planned_pos_world;
    occ_map_->mapToWorld(map_planned_path[i], planned_pos_world);

    geometry_msgs::msg::PoseStamped pose;

    RCLCPP_INFO(
      logger_, "  [%ld]: Map: (%f, %f, %f), World: (%f, %f, %f)", 
      i,
      map_planned_path[i](0), map_planned_path[i](1), map_planned_path[i](2),
      planned_pos_world(0), planned_pos_world(1), planned_pos_world(2));

    pose.pose.position.x = planned_pos_world(0);
    pose.pose.position.y = planned_pos_world(1);
    pose.pose.position.z = planned_pos_world(2);

    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;
    pose.pose.orientation.w = 1.0;

    plan.poses.push_back(pose);
  }

  return !plan.poses.empty();
}

// rcl_interfaces::msg::SetParametersResult
// NavfnPlanner::dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters)
// {
//   rcl_interfaces::msg::SetParametersResult result;
//   for (auto parameter : parameters) {
//     const auto & type = parameter.get_type();
//     const auto & name = parameter.get_name();

//     if (type == ParameterType::PARAMETER_DOUBLE) {
//       if (name == name_ + ".tolerance") {
//         tolerance_ = parameter.as_double();
//       }
//     } else if (type == ParameterType::PARAMETER_BOOL) {
//       if (name == name_ + ".use_astar") {
//         use_astar_ = parameter.as_bool();
//       } else if (name == name_ + ".allow_unknown") {
//         allow_unknown_ = parameter.as_bool();
//       } else if (name == name_ + ".use_final_approach_orientation") {
//         use_final_approach_orientation_ = parameter.as_bool();
//       }
//     }
//   }
//   result.successful = true;
//   return result;
// }

} // namespace astar_planner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(astar_planner::AStarPlanner, gestelt_core::GlobalPlanner)
