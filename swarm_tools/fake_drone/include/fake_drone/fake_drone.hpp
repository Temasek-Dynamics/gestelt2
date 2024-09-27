#ifndef FAKE_DRONE__FAKE_DRONE_HPP_
#define FAKE_DRONE__FAKE_DRONE_HPP_

#include <Eigen/Eigen>

#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/odometry.hpp>

// #include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <tf2_ros/transform_broadcaster.h>

#include <gestelt_interfaces/msg/space_time_path.hpp>

#include <minco_traj_gen/minco_traj_gen.hpp>

// #include "tinysplinecxx.h"  // For spline interpolation

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace std::placeholders;


class FakeDrone : public rclcpp::Node
{
    public:
        FakeDrone();

        virtual ~FakeDrone();

    private:
        /* Timer callbacks */

        // Main timer for updating UAV simulation 
        void stateUpdateTimerCB();

        void tfUpdateTimerCB();

        /* Subscription callbacks */

        void frontEndPlanCB(const gestelt_interfaces::msg::SpaceTimePath::UniquePtr &msg);

        /* Checks */

        /** Helper methods */
        void setStateFromPlan(	const std::vector<Eigen::Vector4d>& space_time_path, 
                                const double& plan_start_t);

        /* Convert from time [s] to space-time units */
        long tToSpaceTimeUnits(const double& t){
            return std::lround(t / t_unit_);
        }

    private:
        /* Publishers, subscribers, timers and services */
	    rclcpp::Subscription<gestelt_interfaces::msg::SpaceTimePath>::SharedPtr fe_plan_sub_;  // Front end plan subscription

        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_; // Publish odometry
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_; // Publish pose

        rclcpp::TimerBase::SharedPtr tf_update_timer_; // Timer for tf broadcast
        rclcpp::TimerBase::SharedPtr state_update_timer_; // Timer for state update and publishing 
            
        rclcpp::CallbackGroup::SharedPtr sim_update_cb_group_;

        /* Params */
        int drone_id_{-1};
        double t_unit_{0.1};     // [s] Time duration of each space-time A* unit

        std::string global_frame_, local_map_frame_, uav_frame_;

        /* Data */
        nav_msgs::msg::Odometry odom_msg_;
        geometry_msgs::msg::PoseStamped pose_msg_;

        bool plan_received_{false}; // indicates that plan is received

        // std::shared_ptr<tinyspline::BSpline> spline_; // Spline formed from interpolating control points of front end path
        std::vector<Eigen::Vector4d> fe_space_time_path_; // Front end space time path

        double plan_start_t_; // [s] Time that plan started

        /* Mutexes  */
        std::mutex state_mutex_;

        // TF transformation 
        std::unique_ptr<tf2_ros::TransformBroadcaster> bl_broadcaster_tf_; // map to base_link tf broadcaster
};

#endif // FAKE_DRONE_HPP_
