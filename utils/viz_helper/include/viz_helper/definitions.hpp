#ifndef _VIZ_HELPER__DEFINITIONS_HPP_
#define _VIZ_HELPER__DEFINITIONS_HPP_

#include <Eigen/Eigen>

namespace viz_helper{

  /* Colors */
  static const Eigen::Vector4d fe_plan_start_color{1.0, 1.0, 0.0, 0.5}; // Front-end start
  static const Eigen::Vector4d fe_plan_goal_color{0.0, 1.0, 0.0, 0.5}; // Front-end goal
  static const Eigen::Vector4d fe_plan_path_color{1.0, 0.5, 0.0, 0.5}; // Front-end path
  static const  Eigen::Vector4d exec_traj_color{1, 51.0/221.0, 51.0/221.0, 0.5}; // #FF3333

} // namespace viz_helper

#endif // _VIZ_HELPER__DEFINITIONS_HPP_
