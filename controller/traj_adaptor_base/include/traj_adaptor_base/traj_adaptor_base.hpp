#ifndef TRAJ_ADAPTOR_BASE__TRAJECTORY_ADAPTOR_HPP
#define TRAJ_ADAPTOR_BASE__TRAJECTORY_ADAPTOR_HPP

#include <Eigen/Eigen>

#include <optimizer/poly_traj_utils.hpp>
#include <traj_utils/PolyTraj.h>


namespace traj_adaptor_base
{
    class TrajectoryAdaptor
    {
        public:
            // Initialize is used to pass paramters to the object
            virtual void initialize() = 0;

            virtual void getPVA() = 0;

        protected:
            //constructor without parameters is required
            TrajectoryAdaptor(){}
    };
}   // namespace traj_adaptor_base

#endif  // TRAJ_ADAPTOR_BASE__TRAJECTORY_ADAPTOR_HPP

