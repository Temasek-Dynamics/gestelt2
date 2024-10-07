
#include <traj_adaptor_plugins/minco.hpp>

namespace traj_adaptor_plugins
{

    Eigen::Vector2d FakeDrone::calculate_yaw(const std::shared_ptr<minco::Trajectory>& traj, 
                                                        const double& t_cur, const double& dt)
    {
        Eigen::Vector2d yaw_yawdot(0, 0);

        // get direction vector
        Eigen::Vector3d dir = t_cur + t_step_ <= traj->getTotalDuration()
                                    ? traj->getPos(t_cur + t_step_) - traj->getPos(t_cur)
                                    : traj->getPos(traj->getTotalDuration()) - traj->getPos(t_cur);

        double yaw_temp = dir.norm() > 0.1
                                ? atan2(dir(1), dir(0))
                                : prev_yaw_;

        double yawdot = 0;
        double d_yaw = yaw_temp - prev_yaw_;
        if (d_yaw >= M_PI)
        {
            d_yaw -= 2 * M_PI;
        }
        if (d_yaw <= -M_PI)
        {
            d_yaw += 2 * M_PI;
        }
        
        // Set maximum values for yaw_dot and yaw_ddot
        const double YDM = d_yaw >= 0 ? YAW_DOT_MAX_PER_SEC : -YAW_DOT_MAX_PER_SEC;
        const double YDDM = d_yaw >= 0 ? YAW_DOT_DOT_MAX_PER_SEC : -YAW_DOT_DOT_MAX_PER_SEC;
        double d_yaw_max;

        if (fabs(prev_yaw_dot_ + dt * YDDM) <= fabs(YDM)) // Within yaw_dot limits
        {
            // yawdot = prev_yaw_dot_ + dt * YDDM;
            d_yaw_max = (prev_yaw_dot_ * dt) + (0.5 * YDDM * dt * dt);
        }
        else // exceed yaw_dot limits
        {
            // yawdot = YDM;
            double t1 = (YDM - prev_yaw_dot_) / YDDM;
            d_yaw_max = ((dt - t1) + dt) * (YDM - prev_yaw_dot_) / 2.0;
        }

        if (fabs(d_yaw) > fabs(d_yaw_max))
        {
            d_yaw = d_yaw_max;
        }
        yawdot = d_yaw / dt;

        double yaw = prev_yaw_ + d_yaw;
        if (yaw > M_PI)
            yaw -= 2 * M_PI;
        if (yaw < -M_PI)
            yaw += 2 * M_PI;
        yaw_yawdot(0) = yaw;
        yaw_yawdot(1) = yawdot;

        prev_yaw_ = yaw_yawdot(0);
        prev_yaw_dot_ = yaw_yawdot(1);

        yaw_yawdot(1) = yaw_temp;

        return yaw_yawdot;
    }



}  // namespace traj_adaptor_plugins

