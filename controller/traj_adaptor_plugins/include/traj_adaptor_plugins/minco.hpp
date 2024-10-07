#ifndef TRAJ_ADAPTOR_PLUGINS__MINCO_HPP_
#define TRAJ_ADAPTOR_PLUGINS__MINCO_HPP_

#include "traj_adaptor_plugins/visibility_control.h"
#include <traj_adaptor_base/trajectory_adaptor.hpp>

namespace traj_adaptor_plugins
{

class MINCO : public traj_adaptor_base::TrajectoryAdaptor
{
  public:
    void initialize() override
    {
    }
  protected:
};

} // namespace traj_adaptor_plugins

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(traj_adaptor_plugins::MINCO, 
                        traj_adaptor_base::TrajectoryAdaptor)

#endif  // TRAJ_ADAPTOR_PLUGINS__MINCO_HPP_
