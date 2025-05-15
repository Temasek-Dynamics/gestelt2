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

#ifndef FAKE_DRONE__FAKE_DRONE_HPP_
#define FAKE_DRONE__FAKE_DRONE_HPP_

#include <Eigen/Eigen>

#include <rclcpp/rclcpp.hpp>

#include <tf2_ros/transform_broadcaster.h>

#include <mavros_msgs/msg/position_target.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/set_mode.hpp>

#include <nav_msgs/msg/odometry.hpp>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace std::placeholders;

class FakeDrone : public rclcpp::Node
{
public:
    FakeDrone();

    virtual ~FakeDrone();

private:

    /* Subscription callbacks */
    void trajectorySetpointSubCB(const mavros_msgs::msg::PositionTarget::UniquePtr &msg);

    /* Service callbacks */

    void cmdArmingSrvCB(const std::shared_ptr<mavros_msgs::srv::CommandBool::Request> req,
                        std::shared_ptr<mavros_msgs::srv::CommandBool::Response> resp);

    void setModeSrvCB(const std::shared_ptr<mavros_msgs::srv::SetMode::Request> req,
                        std::shared_ptr<mavros_msgs::srv::SetMode::Response> resp);

    /* Timer callbacks */
    
    // Updates state of UAV
    void stateUpdateTimerCB(); 

    // Publishes odometry and broadcasts transform of UAV base link relative to a global reference
    void odomUpdateTimerCB();

    /* Checks */

    /** Helper methods */

    /* Convert from RPY to quaternion */
    Eigen::Quaterniond RPYToQuaternion(const double& roll, const double& pitch, const double& yaw){
        return Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) 
                * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
                * Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
    }

    // Eigen::Vector3d vecENUToNED(const Eigen::Vector3d &vec)
    // {
    //     return Eigen::PermutationMatrix<3>(Eigen::Vector3i(1, 0, 2)) 
    //         * (Eigen::DiagonalMatrix<double, 3>(1, 1, -1) * vec);
    // }

private:
    /* Publishers, subscribers, timers and services */
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_; 
    rclcpp::Publisher<mavros_msgs::msg::State>::SharedPtr vehicle_state_pub_; 

    rclcpp::Subscription<mavros_msgs::msg::PositionTarget>::SharedPtr 
        trajectory_setpoint_sub_;
    
    rclcpp::TimerBase::SharedPtr tf_broadcast_timer_; // Timer for tf broadcast
    rclcpp::TimerBase::SharedPtr state_update_timer_; // Timer for state update and publishing 
    rclcpp::TimerBase::SharedPtr odom_update_timer_; // Timer for state update and publishing 
        
    rclcpp::CallbackGroup::SharedPtr sim_update_cb_group_;
    rclcpp::CallbackGroup::SharedPtr traj_sp_cb_group_;

    rclcpp::Service<mavros_msgs::srv::CommandBool>::SharedPtr cmd_arming_srv_;
    rclcpp::Service<mavros_msgs::srv::SetMode>::SharedPtr set_mode_srv_;

    /* Params */
    int drone_id_{-1};

    /* Data */
    std::unique_ptr<mavros_msgs::msg::State> vehicle_state_msg_;
    std::unique_ptr<nav_msgs::msg::Odometry> odom_msg_;
    geometry_msgs::msg::TransformStamped map_to_bl_tf_;

    /* Mutexes  */
    std::mutex state_mutex_;

    /* TF */
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_; 
};

#endif // FAKE_DRONE_HPP_
