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

#ifndef POLY_TRAJ_CMD_HPP_
#define POLY_TRAJ_CMD_HPP_

#include <rclcpp/rclcpp.hpp>
#include <minco_interfaces/msg/polynomial_trajectory.hpp>
#include <minco_traj_gen/minco_traj_gen.hpp>
// #include <minco_interfaces/msg/minco_trajectory.hpp>

// using namespace std::chrono;
// using namespace std::chrono_literals;
// using namespace px4_msgs::msg;
using namespace std::placeholders;

class PolyTrajCmd 
{
public:
	/** Initialization methods */
	PolyTrajCmd(rclcpp::Node::SharedPtr node)
	: node_(node)
	{
		minco_traj_sub_ = node_->create_subscription<minco_interfaces::msg::PolynomialTrajectory>(
			"poly_traj", rclcpp::SensorDataQoS(),
			std::bind(&PolyTrajCmd::PolynomialTrajSubCB, this, _1));
	}

	~PolyTrajCmd(){}

	/* Get position, yaw_yawrate, velocity, acceleration*/
	/* getCmd(pos_cmd_, yaw_yawrate_cmd_, vel_cmd_, acc_cmd_)*/
	bool getCmd(Eigen::Vector3d& pos, 
				Eigen::Vector2d& yaw_yawrate,
				Eigen::Vector3d& vel,
				Eigen::Vector3d& acc)
	{
		if (traj_ == nullptr){
			return false;
		}

		auto t_now = node_->get_clock()->now();
		double e_t_start = t_now.seconds() - traj_->getGlobalStartTime();			 	// [s] Elapsed time since plan start

		if (e_t_start < 0.0){
			// std::cout << "Trajectory start time is in the future" << std::endl;
			return false;
		}
		else if (e_t_start >= traj_->getTotalDuration()){
			// std::cout << "Trajectory has finished executing in the past" << std::endl;
			return false;
		}

		{
			std::lock_guard<std::mutex> traj_mtx_guard(traj_mtx_);

			pos = traj_->getPos(e_t_start);
			vel = traj_->getVel(e_t_start);
			acc = traj_->getAcc(e_t_start);
			// jer = traj_->getJer(e_t_start);
			yaw_yawrate = calculate_yaw(traj_, 
										yaw_yawrate,
										e_t_start, t_now.seconds() - t_last_traj_samp_);
		}

		t_last_traj_samp_ = t_now.seconds();

		return true;
	}


private:
	void PolynomialTrajSubCB(const minco_interfaces::msg::PolynomialTrajectory::UniquePtr msg)
	{
		if (msg->order != 5)
		{
			std::cout << "Only support polynomial trajectory of 5th Order!" << std::endl;
			return;
		}
		if (msg->duration.size() * (msg->order + 1) != msg->coef_x.size())
		{
			std::cout << "Invalid polynomial trajectory params" << std::endl;
			return;
		}

		// Convert the polynomial trajectory message into minco::Trajectory type and storing it

		size_t N = msg->duration.size(); // number of trajectory segments 
		std::vector<double> ts(N); // vector of segment time durations
		std::vector<minco::CoefficientMat> cMats(N);	// Coefficient matrices
		for (size_t i = 0; i < N; i++)	// For each segment
		{
			int i6 = i * 6;
			cMats[i].row(0) <<  msg->coef_x[i6 + 0], msg->coef_x[i6 + 1], msg->coef_x[i6 + 2],
								msg->coef_x[i6 + 3], msg->coef_x[i6 + 4], msg->coef_x[i6 + 5];
			cMats[i].row(1) <<  msg->coef_y[i6 + 0], msg->coef_y[i6 + 1], msg->coef_y[i6 + 2],
								msg->coef_y[i6 + 3], msg->coef_y[i6 + 4], msg->coef_y[i6 + 5];
			cMats[i].row(2) <<  msg->coef_z[i6 + 0], msg->coef_z[i6 + 1], msg->coef_z[i6 + 2],
								msg->coef_z[i6 + 3], msg->coef_z[i6 + 4], msg->coef_z[i6 + 5];

			ts[i] = msg->duration[i];
		}

		{
			std::lock_guard<std::mutex> traj_mtx_guard(traj_mtx_);
				
			traj_ = std::make_shared<minco::Trajectory>(ts, cMats);
			traj_->setGlobalStartTime(msg->start_time);
		}
	}

	Eigen::Vector2d calculate_yaw(const std::shared_ptr<minco::Trajectory>& traj, 
											 const Eigen::Vector2d prev_yaw_yawdot,
											 const double& t_cur, const double& dt)
	{
		Eigen::Vector2d yaw_yawrate(0, 0);

		// get direction vector
		Eigen::Vector3d dir = t_cur + t_step_ <= traj->getTotalDuration()
									? traj->getPos(t_cur + t_step_) - traj->getPos(t_cur)
									: traj->getPos(traj->getTotalDuration()) - traj->getPos(t_cur);

		double yaw_temp = dir.norm() > 0.1
								? atan2(dir(1), dir(0))
								: prev_yaw_yawdot(0);

		double yawdot = 0;
		double d_yaw = yaw_temp - prev_yaw_yawdot(0);
		if (d_yaw >= M_PI)
		{
			d_yaw -= 2 * M_PI;
		}
		if (d_yaw <= -M_PI)
		{
			d_yaw += 2 * M_PI;
		}
		
		// Set maximum values for yawrate and yaw_ddot
		const double YDM = d_yaw >= 0 ? YAWRATE_MAX_PER_SEC : -YAWRATE_MAX_PER_SEC;
		const double YDDM = d_yaw >= 0 ? YAWRATE_DOT_MAX_PER_SEC : -YAWRATE_DOT_MAX_PER_SEC;
		double d_yaw_max;

		if (fabs(prev_yaw_yawdot(1) + dt * YDDM) <= fabs(YDM)) // Within yawrate limits
		{
			// yawdot = prev_yaw_yawdot(1) + dt * YDDM;
			d_yaw_max = (prev_yaw_yawdot(1) * dt) + (0.5 * YDDM * dt * dt);
		}
		else // exceed yawrate limits
		{
			// yawdot = YDM;
			double t1 = (YDM - prev_yaw_yawdot(1)) / YDDM;
			d_yaw_max = ((dt - t1) + dt) * (YDM - prev_yaw_yawdot(1)) / 2.0;
		}

		if (fabs(d_yaw) > fabs(d_yaw_max))
		{
			d_yaw = d_yaw_max;
		}
		yawdot = d_yaw / dt;
		
		double yaw = prev_yaw_yawdot(0) + d_yaw;
		/* Correct phase*/
		if (yaw > M_PI)
			yaw -= 2 * M_PI;
		if (yaw < -M_PI)
			yaw += 2 * M_PI;
		
		yaw_yawrate(0) = yaw;
		yaw_yawrate(1) = yawdot;

		return yaw_yawrate;
	}

	private:
  		rclcpp::Node::SharedPtr node_;
		rclcpp::Subscription<minco_interfaces::msg::PolynomialTrajectory>::SharedPtr minco_traj_sub_;

		std::shared_ptr<minco::Trajectory> traj_; 

		/* Params */
        const double YAWRATE_MAX_PER_SEC{2 * M_PI};
        const double YAWRATE_DOT_MAX_PER_SEC{5 * M_PI};
		double t_step_{1.0}; // [s] Forward time step along trajectory used to obtain yaw and yawrate

        double t_last_traj_samp_{0.0};

		std::mutex traj_mtx_; // Mutex lock for trajectory
};

#endif //POLY_TRAJ_CMD_HPP_