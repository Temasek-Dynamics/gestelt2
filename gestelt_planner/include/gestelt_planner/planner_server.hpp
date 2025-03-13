/****************************************************************************
 * MIT License
 *  
 *	Copyright (c) 2024 John Tan. All rights reserved.
 *
 *	Permission is hereby granted, free of charge, to any person obtaining a copy
 *	of this software and associated documentation files (the "Software"), to deal
 *	in the Software without restriction, including without limitation the rights
 *	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *	copies of the Software, and to permit persons to whom the Software is
 *	furnished to do so, subject to the following conditions:
 *
 *	The above copyright notice and this permission notice shall be included in all
 *	copies or substantial portions of the Software.
 *
 *	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *	SOFTWARE.
 *
 ****************************************************************************/

 #ifndef GESTELT_PLANNER__PLANNER_SERVER_HPP_
 #define GESTELT_PLANNER__PLANNER_SERVER_HPP_
 
 #include <Eigen/Eigen>
 
 #include <queue>
 
 #include <rclcpp/rclcpp.hpp>
 
 #include <tf2_ros/transform_listener.h>
 #include <tf2_ros/buffer.h>
 #include <tf2/exceptions.h>
 
 #include <std_msgs/msg/empty.hpp>
 
 #include "nav2_util/lifecycle_node.hpp"
 
 #include <logger_wrapper/logger_wrapper.hpp>
 #include <logger_wrapper/timer.hpp>
 
 namespace gestelt_planner
 {
 class PlannerServer : public nav2_util::LifecycleNode
 {
 public:
   /**
    * @brief A constructor for gestelt_planner::PlannerServer
    * @param options Additional options to control creation of the node.
    */
   explicit PlannerServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
 
   /**
    * @brief A destructor for nav2_planner::PlannerServer
    */
   ~PlannerServer();
 
   using PlannerMap = std::unordered_map<std::string, nav2_core::GlobalPlanner::Ptr>;
 
   /**
    * @brief Method to get plan from the desired plugin
    * @param start starting pose
    * @param goal goal request
    * @param planner_id The planner to plan with
    * @param cancel_checker A function to check if the action has been canceled
    * @return Path
    */
   nav_msgs::msg::Path getPlan(
     const geometry_msgs::msg::PoseStamped & start,
     const geometry_msgs::msg::PoseStamped & goal,
     const std::string & planner_id,
     std::function<bool()> cancel_checker);
 
 protected:
   /**
    * @brief Configure member variables and initializes planner
    * @param state Reference to LifeCycle node state
    * @return SUCCESS or FAILURE
    */
   nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
   /**
    * @brief Activate member variables
    * @param state Reference to LifeCycle node state
    * @return SUCCESS or FAILURE
    */
   nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
   /**
    * @brief Deactivate member variables
    * @param state Reference to LifeCycle node state
    * @return SUCCESS or FAILURE
    */
   nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
   /**
    * @brief Reset member variables
    * @param state Reference to LifeCycle node state
    * @return SUCCESS or FAILURE
    */
   nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
   /**
    * @brief Called when in shutdown state
    * @param state Reference to LifeCycle node state
    * @return SUCCESS or FAILURE
    */
   nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;
 
   using ActionToPose = nav2_msgs::action::ComputePathToPose;
   using ActionToPoseResult = ActionToPose::Result;
   using ActionThroughPoses = nav2_msgs::action::ComputePathThroughPoses;
   using ActionThroughPosesResult = ActionThroughPoses::Result;
   using ActionServerToPose = nav2_util::SimpleActionServer<ActionToPose>;
   using ActionServerThroughPoses = nav2_util::SimpleActionServer<ActionThroughPoses>;
 
 
   /**
    * @brief Check if an action server is valid / active
    * @param action_server Action server to test
    * @return SUCCESS or FAILURE
    */
   template<typename T>
   bool isServerInactive(std::unique_ptr<nav2_util::SimpleActionServer<T>> & action_server);
 
   /**
    * @brief Check if an action server has a cancellation request pending
    * @param action_server Action server to test
    * @return SUCCESS or FAILURE
    */
   template<typename T>
   bool isCancelRequested(std::unique_ptr<nav2_util::SimpleActionServer<T>> & action_server);
 
   /**
    * @brief Wait for costmap to be valid with updated sensor data or repopulate after a
    * clearing recovery. Blocks until true without timeout.
    */
   void waitForCostmap();
 
   /**
    * @brief Check if an action server has a preemption request and replaces the goal
    * with the new preemption goal.
    * @param action_server Action server to get updated goal if required
    * @param goal Goal to overwrite
    */
   template<typename T>
   void getPreemptedGoalIfRequested(
     std::unique_ptr<nav2_util::SimpleActionServer<T>> & action_server,
     typename std::shared_ptr<const typename T::Goal> goal);
 
   /**
    * @brief Get the starting pose from costmap or message, if valid
    * @param action_server Action server to terminate if required
    * @param goal Goal to find start from
    * @param start The starting pose to use
    * @return bool If successful in finding a valid starting pose
    */
   template<typename T>
   bool getStartPose(
     typename std::shared_ptr<const typename T::Goal> goal,
     geometry_msgs::msg::PoseStamped & start);
 
   /**
    * @brief Transform start and goal poses into the costmap
    * global frame for path planning plugins to utilize
    * @param start The starting pose to transform
    * @param goal Goal pose to transform
    * @return bool If successful in transforming poses
    */
   bool transformPosesToGlobalFrame(
     geometry_msgs::msg::PoseStamped & curr_start,
     geometry_msgs::msg::PoseStamped & curr_goal);
 
   /**
    * @brief Validate that the path contains a meaningful path
    * @param action_server Action server to terminate if required
    * @param goal Goal Current goal
    * @param path Current path
    * @param planner_id The planner ID used to generate the path
    * @return bool If path is valid
    */
   template<typename T>
   bool validatePath(
     const geometry_msgs::msg::PoseStamped & curr_goal,
     const nav_msgs::msg::Path & path,
     const std::string & planner_id);
 
   /**
    * @brief The action server callback which calls planner to get the path
    * ComputePathToPose
    */
   void computePlan();
 
   /**
    * @brief The action server callback which calls planner to get the path
    * ComputePathThroughPoses
    */
   void computePlanThroughPoses();
 
   /**
    * @brief The service callback to determine if the path is still valid
    * @param request to the service
    * @param response from the service
    */
   void isPathValid(
     const std::shared_ptr<nav2_msgs::srv::IsPathValid::Request> request,
     std::shared_ptr<nav2_msgs::srv::IsPathValid::Response> response);
 
   /**
    * @brief Publish a path for visualization purposes
    * @param path Reference to Global Path
    */
   void publishPlan(const nav_msgs::msg::Path & path);
 
   void exceptionWarning(
     const geometry_msgs::msg::PoseStamped & start,
     const geometry_msgs::msg::PoseStamped & goal,
     const std::string & planner_id,
     const std::exception & ex);
 
   /**
    * @brief Callback executed when a parameter change is detected
    * @param event ParameterEvent message
    */
   rcl_interfaces::msg::SetParametersResult
   dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);
 
   // Our action server implements the ComputePathToPose action
   std::unique_ptr<ActionServerToPose> action_server_pose_;
   std::unique_ptr<ActionServerThroughPoses> action_server_poses_;
 
   // Dynamic parameters handler
   rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;
   std::mutex dynamic_params_lock_;
 
   // Planner
   PlannerMap planners_;
   pluginlib::ClassLoader<gestelt_core::GlobalPlanner> gp_loader_;
   std::vector<std::string> default_ids_;
   std::vector<std::string> default_types_;
   std::vector<std::string> planner_ids_;
   std::vector<std::string> planner_types_;
   double max_planner_duration_;
   rclcpp::Duration costmap_update_timeout_;
   std::string planner_ids_concat_;
 
   // TF buffer
   std::shared_ptr<tf2_ros::Buffer> tf_;
 
   // Global Costmap
//    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
//    std::unique_ptr<nav2_util::NodeThread> costmap_thread_;
//    nav2_costmap_2d::Costmap2D * costmap_;
 
   // Publishers for the path
   rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr plan_publisher_;
 
   // Service to determine if the path is valid
   rclcpp::Service<nav2_msgs::srv::IsPathValid>::SharedPtr is_path_valid_service_;
 
 };
 
} // namespace gestelt_planner

#endif // GESTELT_PLANNER__PLANNER_SERVER_HPP_