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

#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace std::placeholders;

using namespace px4_msgs::msg;

enum PX4_CUSTOM_MAIN_MODE {
	PX4_CUSTOM_MAIN_MODE_MANUAL = 1,
	PX4_CUSTOM_MAIN_MODE_ALTCTL, // 2
	PX4_CUSTOM_MAIN_MODE_POSCTL,  // 3
	PX4_CUSTOM_MAIN_MODE_AUTO, // 4
	PX4_CUSTOM_MAIN_MODE_ACRO, // 5
	PX4_CUSTOM_MAIN_MODE_OFFBOARD,  // 6
	PX4_CUSTOM_MAIN_MODE_STABILIZED,
	PX4_CUSTOM_MAIN_MODE_RATTITUDE_LEGACY,
	PX4_CUSTOM_MAIN_MODE_SIMPLE, /* unused, but reserved for future use */
	PX4_CUSTOM_MAIN_MODE_TERMINATION
};

enum PX4_CUSTOM_SUB_MODE_AUTO {
	PX4_CUSTOM_SUB_MODE_AUTO_READY = 1,
	PX4_CUSTOM_SUB_MODE_AUTO_TAKEOFF,
	PX4_CUSTOM_SUB_MODE_AUTO_LOITER,
	PX4_CUSTOM_SUB_MODE_AUTO_MISSION,
	PX4_CUSTOM_SUB_MODE_AUTO_RTL,
	PX4_CUSTOM_SUB_MODE_AUTO_LAND,
	PX4_CUSTOM_SUB_MODE_AUTO_RESERVED_DO_NOT_USE, // was PX4_CUSTOM_SUB_MODE_AUTO_RTGS, deleted 2020-03-05
	PX4_CUSTOM_SUB_MODE_AUTO_FOLLOW_TARGET,
	PX4_CUSTOM_SUB_MODE_AUTO_PRECLAND,
	PX4_CUSTOM_SUB_MODE_AUTO_VTOL_TAKEOFF,
	PX4_CUSTOM_SUB_MODE_EXTERNAL1,
	PX4_CUSTOM_SUB_MODE_EXTERNAL2,
	PX4_CUSTOM_SUB_MODE_EXTERNAL3,
	PX4_CUSTOM_SUB_MODE_EXTERNAL4,
	PX4_CUSTOM_SUB_MODE_EXTERNAL5,
	PX4_CUSTOM_SUB_MODE_EXTERNAL6,
	PX4_CUSTOM_SUB_MODE_EXTERNAL7,
	PX4_CUSTOM_SUB_MODE_EXTERNAL8
};

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

        void trajectorySetpointSubCB(const TrajectorySetpoint::UniquePtr &msg);
        void offboardCtrlModeSubCB(const OffboardControlMode::UniquePtr &msg);
        void VehicleCommandSubCB(const VehicleCommand::UniquePtr &msg);

        /* Checks */

        /** Helper methods */

        /* Convert from RPY to quaternion */
        Eigen::Quaterniond RPYToQuaternion(const double& roll, const double& pitch, const double& yaw){
            return Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) 
                    * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
                    * Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
        }

        Eigen::Vector3d vecENUToNED(const Eigen::Vector3d &vec)
        {
            return Eigen::PermutationMatrix<3>(Eigen::Vector3i(1, 0, 2)) 
                * (Eigen::DiagonalMatrix<double, 3>(1, 1, -1) * vec);
        }

    private:
        /* Publishers, subscribers, timers and services */
        rclcpp::Publisher<VehicleOdometry>::SharedPtr fcu_odom_pub_; 
        rclcpp::Publisher<VehicleStatus>::SharedPtr vehicle_status_pub_; 

        rclcpp::Subscription<OffboardControlMode>::SharedPtr offboard_control_mode_sub_;
        rclcpp::Subscription<TrajectorySetpoint>::SharedPtr trajectory_setpoint_sub_;
        rclcpp::Subscription<VehicleCommand>::SharedPtr vehicle_command_sub_;
        
        rclcpp::TimerBase::SharedPtr tf_update_timer_; // Timer for tf broadcast
        rclcpp::TimerBase::SharedPtr state_update_timer_; // Timer for state update and publishing 
            
        rclcpp::CallbackGroup::SharedPtr sim_update_cb_group_;
        rclcpp::CallbackGroup::SharedPtr fcu_cb_group_;
        rclcpp::CallbackGroup::SharedPtr traj_sp_cb_group_;

        /* Params */
        int drone_id_{-1};

        /* Data */
        std::unique_ptr<VehicleStatus> vehicle_status_msg_;
        std::unique_ptr<VehicleOdometry> vehicle_odom_msg_;

        /* Mutexes  */
        std::mutex state_mutex_;

};

#endif // FAKE_DRONE_HPP_
