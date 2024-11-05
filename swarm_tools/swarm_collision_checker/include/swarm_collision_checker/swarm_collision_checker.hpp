#ifndef _SWARM_COLLISION_CHECKER_H_
#define _SWARM_COLLISION_CHECKER_H_

#include <rclcpp/rclcpp.hpp>
#include <Eigen/Eigen>

#include <visualization_msgs/msg/marker.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include "nanoflann.hpp" // For nearest neighbors queries
#include "KDTreeVectorOfVectorsAdaptor.h" // For nearest neighbors queries

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace std::placeholders;

class SwarmCollisionChecker : public rclcpp::Node
{
public:
  SwarmCollisionChecker(): Node("swarm_collision_checker")
  {
    reentrant_cb_group_ = this->create_callback_group(
      rclcpp::CallbackGroupType::Reentrant);

	  this->declare_parameter("num_drones", 0);
	  this->declare_parameter("odom_topic", "odom");
	  this->declare_parameter("collision_check.frequency", 10.0);
	  this->declare_parameter("collision_check.warn_radius", 0.225);
	  this->declare_parameter("collision_check.fatal_radius", 0.14);

    num_drones_ = this->get_parameter("num_drones").as_int();
    std::string odom_topic = this->get_parameter("odom_topic").as_string();
    double col_check_freq = this->get_parameter("collision_check.frequency").as_double();
    col_warn_radius_ = this->get_parameter("collision_check.warn_radius").as_double();
    col_fatal_radius_ = this->get_parameter("collision_check.fatal_radius").as_double();

    collision_viz_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
      "/swarm_collision_checker/collisions", 10);

    drone_poses_ = std::make_unique<std::vector<Eigen::Vector3d>>();

    // Subscribers
    auto reentrant_cb_opt = rclcpp::SubscriptionOptions();
    reentrant_cb_opt.callback_group = reentrant_cb_group_;

    for (int i = 0; i < num_drones_; i++){
      std::function<void(const nav_msgs::msg::Odometry::UniquePtr msg)> bound_callback_func =
        std::bind(&SwarmCollisionChecker::odomCB, this, _1, i);

      drones_pose_subs_.push_back(
        this->create_subscription<nav_msgs::msg::Odometry>(
          "/d"+std::to_string(i)+ "/" + odom_topic, 
          rclcpp::SensorDataQoS(), 
          bound_callback_func, 
          reentrant_cb_opt)
      );

      (*drone_poses_).push_back(Eigen::Vector3d::Constant(999.9));
    }

    // TODO: sleep
    rclcpp::Rate loop_rate(1000);
    loop_rate.sleep();

    // Timers
    check_collision_timer_ = this->create_wall_timer((1.0/col_check_freq) *1000ms, 
      std::bind(&SwarmCollisionChecker::checkCollisionTimerCb, this), reentrant_cb_group_);

    std::cout << "[swarm_collision_checker]: Initialized" << std::endl;
  }

private: 
  // Subscribe to robot pose
  void odomCB(const nav_msgs::msg::Odometry::UniquePtr &msg, int drone_id)
  {
    std::lock_guard<std::mutex> poses_mtx_grd(poses_mtx_);
    // ROS_INFO("[SwarmCollisionChecker]: Pose callback for drone %d", drone_id);
    (*drone_poses_)[drone_id] = Eigen::Vector3d{msg->pose.pose.position.x,  
                                                msg->pose.pose.position.y,  
                                                msg->pose.pose.position.z};
  }


  // Timer callback for checking collision between swarm agentss
  void checkCollisionTimerCb()
  {

    const size_t        num_closest = 1;
    std::vector<size_t> out_indices(num_closest);
    std::vector<double> out_distances_sq(num_closest);

    for (int i = 0; i < num_drones_; i++){ 
      std::lock_guard<std::mutex> poses_mtx_grd(poses_mtx_);

      // For every drone, get the nearest neighbor and see if distance is within tolerance
      // If not, publish a collision sphere.

      std::vector<double> query_pt(3);
      query_pt[0] = (*drone_poses_)[i](0);
      query_pt[1] = (*drone_poses_)[i](1);
      query_pt[2] = (*drone_poses_)[i](2);

      std::vector<Eigen::Vector3d> drone_poses_wo_self = *drone_poses_;
      drone_poses_wo_self.erase(drone_poses_wo_self.begin() + i);
      drone_poses_kdtree_ = 
          std::make_unique<KDTreeVectorOfVectorsAdaptor<std::vector<Eigen::Vector3d>, double>>(
              3, drone_poses_wo_self);

      drone_poses_kdtree_->query(&query_pt[0], num_closest, &out_indices[0], &out_distances_sq[0]);

      double dist_to_nearest_drone = sqrt(out_distances_sq[0]);

      if (dist_to_nearest_drone <= col_warn_radius_){
        // Take average of the 2 drones' positions
        Eigen::Vector3d drone_nearest_pos = 0.5 * (drone_poses_wo_self[out_indices[0]] + (*drone_poses_)[i]);

        publishCollisionSphere(drone_nearest_pos, dist_to_nearest_drone, col_fatal_radius_, col_warn_radius_);
      }
    }
  }

  // Publish sphere representing collision
  void publishCollisionSphere(  
    const Eigen::Vector3d &pos, const double& dist_to_obs, 
    const double& fatal_radius, const double& warn_radius)
  {
    static int col_viz_id = 0;

    visualization_msgs::msg::Marker sphere;
    sphere.header.frame_id = "world";
    sphere.header.stamp = this->get_clock()->now();
    sphere.type = visualization_msgs::msg::Marker::SPHERE;
    sphere.action = visualization_msgs::msg::Marker::ADD;
    sphere.ns = "swarm_collision";
    sphere.pose.orientation.w = 1.0;
    sphere.id = col_viz_id++;

    // Make the alpha and red color value scale from 0.0 to 1.0 depending on the distance to the obstacle. 
    // With the upper limit being the warn_radius, and the lower limit being the fatal_radius
    double fatal_ratio = std::clamp((warn_radius - dist_to_obs)/(warn_radius - fatal_radius), 0.0, 1.001);

    if (fatal_ratio >= 1.0){
      sphere.color.r = 1.0;
      sphere.color.g = 0.0;
      sphere.color.b = 0.0; 
      sphere.color.a = 0.8;
    }
    else {
      // Goes from pink to orange
      sphere.color.r = 1.0;
      sphere.color.g = 0.5;
      sphere.color.b = 1.0 - fatal_ratio*(1.0); // If fatal, make blue value 0.0, so sphere is orange.
      sphere.color.a = 0.3 + (fatal_ratio*(0.8-0.3));
    }

    double scale = fatal_ratio < 1.0 ? 0.35 : 0.6;

    sphere.scale.x = scale;
    sphere.scale.y = scale;
    sphere.scale.z = scale;
    sphere.pose.position.x = pos(0);
    sphere.pose.position.y = pos(1);
    sphere.pose.position.z = pos(2);

    collision_viz_pub_->publish(sphere);
  }

private:
	rclcpp::CallbackGroup::SharedPtr reentrant_cb_group_;

  /* ROS objects */
  rclcpp::TimerBase::SharedPtr check_collision_timer_; // Timer for querying KDTree to check collision
	std::vector<rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr>
    drones_pose_subs_;  // Vector of subscribers to drone pose
  
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr collision_viz_pub_; // start and goal visualization publisher

  /* Data structs */
  std::unique_ptr<KDTreeVectorOfVectorsAdaptor<std::vector<Eigen::Vector3d>, double>>   
    drone_poses_kdtree_; // KD Tree for drone poses

  std::unique_ptr<std::vector<Eigen::Vector3d>> drone_poses_;

  /* Params */

  int num_drones_{-1};

  double col_warn_radius_{-1.0};
  double col_fatal_radius_{-1.0};

  /* Params */
  std::mutex poses_mtx_;

}; // class SwarmCollisionChecker

#endif // _SWARM_COLLISION_CHECKER_H_
