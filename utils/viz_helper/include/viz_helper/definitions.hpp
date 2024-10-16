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
