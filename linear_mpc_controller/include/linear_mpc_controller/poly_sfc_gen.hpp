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

#ifndef _SFC__POLYTOPE_SFC_H_
#define _SFC__POLYTOPE_SFC_H_

#include <queue>
#include <unordered_map>

#include <Eigen/Eigen>

#include <decomp_geometry/polyhedron.h>
#include <decomp_util/ellipsoid_decomp.h> 
#include <decomp_ros_msgs/msg/ellipsoid_array.hpp>
#include <decomp_ros_msgs/msg/polyhedron_array.hpp>

#include <geometry_msgs/msg/point.hpp>

namespace sfc
{

struct PolytopeSFCParams{
  double bbox_x{1.0}; // [m] Bounding box x
  double bbox_y{2.0}; // [m] Bounding box y
  double bbox_z{1.0}; // [m] Bounding box z

  int plan_samp_intv{5}; // SFC sampling interval of front-end path
}; // struct PolytopeSFCParams

class PolytopeSFC 
{
public: // Public structs

  // Constructor
  PolytopeSFC(const PolytopeSFCParams& sfc_params): params_(sfc_params)
  {}

  // Reset all data structures
  void reset()
  {
    poly_vec_.clear();
    poly_constr_vec_.clear();
  }

  /**
   * @brief Generate a spherical safe flight corridor given a path
   * 
   * @param path 
   * @return true 
   * @return false 
   */
  bool generateSFC(const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> &obs_pts,
                   const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> &path_3d)
  {
    EllipsoidDecomp3D ellip_decomp_util; // Decomposition util for Liu's method
    
    //Using ellipsoid decomposition
    ellip_decomp_util.set_obs(obs_pts);
    ellip_decomp_util.set_local_bbox(Vec3f(params_.bbox_x, params_.bbox_y, params_.bbox_z)); 
    ellip_decomp_util.dilate(path_3d); // Set max iteration number of 10, do fix the path

    poly_vec_ = ellip_decomp_util.get_polyhedrons();

    // std::vector<LinearConstraint3D> poly_constr_vec_new;

    // // Construct poly_constr_vec
    // for (const auto& poly: poly_vec_)
    // {
    //     int num_planes = poly.vs_.size(); // Num of planes from polyhedron
    //     // Constraint: A_poly * x - b_poly <= 0
    //     MatDNf<3> A_poly(num_planes, 3);        
    //     VecDf b_poly(num_planes);               

    //     for (int i = 0; i < num_planes; i++) { // For each plane
    //         A_poly.row(i) = poly.vs_[i].n_;                  // normal (a,b,c) as in ax+by+cz+d
    //         b_poly(i) = poly.vs_[i].p_.dot(poly.vs_[i].n_);  // Scalar d obtained from point.dot(normal)
    //     }
    //     poly_constr_vec_new.push_back(LinearConstraint3D(A_poly, b_poly));
    // }

    // poly_constr_vec_ = poly_constr_vec_new;

    return true;
  }

public:

  /* Getter methods */
  int getPlanSampleInterval() const
  {
    return params_.plan_samp_intv;
  }
  
  decomp_ros_msgs::msg::PolyhedronArray toSFCMsg(std::string frame)
  {
    decomp_ros_msgs::msg::PolyhedronArray poly_msg = 
      polyhedron_array_to_ros(poly_vec_);
    poly_msg.header.frame_id = frame;

    return poly_msg;
  }

  std::vector<Polyhedron3D, Eigen::aligned_allocator<Polyhedron3D>> getPolyVec() const
  {
    return poly_vec_;
  }

  std::vector<LinearConstraint3D> getPolyConstrVec() const
  {
    return poly_constr_vec_;
  }

private: // Helper methods
  decomp_ros_msgs::msg::Polyhedron polyhedron_to_ros(const Polyhedron3D& poly);

  template <int Dim>
  decomp_ros_msgs::msg::PolyhedronArray polyhedron_array_to_ros(const vec_E<Polyhedron<Dim>>& vs);

  template <int Dim>
  decomp_ros_msgs::msg::EllipsoidArray ellipsoid_array_to_ros(const vec_E<Ellipsoid<Dim>>& Es);

private: // Private members
  /* Params */
  PolytopeSFCParams params_; // SFC parameters

  /* Data structs */
  std::vector<Polyhedron3D, Eigen::aligned_allocator<Polyhedron3D>> poly_vec_; // Vector of polyhedrons represented as hyperplanes
  std::vector<LinearConstraint3D> poly_constr_vec_;   // Vector of Polytope constraints (In the form of Ax = b)
  
}; // class PolytopeSFC

inline decomp_ros_msgs::msg::Polyhedron PolytopeSFC::polyhedron_to_ros(const Polyhedron3D& poly){
  decomp_ros_msgs::msg::Polyhedron msg;
  for (const auto &p : poly.hyperplanes()) {
    geometry_msgs::msg::Point pt, n;
    // Points
    pt.x = p.p_(0);
    pt.y = p.p_(1);
    pt.z = p.p_(2);
    // Normal
    n.x = p.n_(0);
    n.y = p.n_(1);
    n.z = p.n_(2);
    msg.points.push_back(pt);
    msg.normals.push_back(n);
  }

  return msg;
}

template <int Dim>
decomp_ros_msgs::msg::PolyhedronArray PolytopeSFC::polyhedron_array_to_ros(const vec_E<Polyhedron<Dim>>& vs){
  decomp_ros_msgs::msg::PolyhedronArray msg;
  for (const auto &v : vs){
    msg.polyhedrons.push_back(polyhedron_to_ros(v));
  }
  return msg;
}

template <int Dim>
decomp_ros_msgs::msg::EllipsoidArray PolytopeSFC::ellipsoid_array_to_ros(const vec_E<Ellipsoid<Dim>>& Es) {
  decomp_ros_msgs::msg::EllipsoidArray ellipsoids_arr;
  for (unsigned int i = 0; i < Es.size(); i++) {
    decomp_ros_msgs::msg::Ellipsoid ellipsoid;
    auto d = Es[i].d();
    ellipsoid.d[0] = d(0);
    ellipsoid.d[1] = d(1);
    ellipsoid.d[2] = Dim == 2 ? 0:d(2);

    auto C = Es[i].C();
    for (int x = 0; x < 3; x++) {
      for (int y = 0; y < 3; y++) {
        if(x < Dim && y < Dim)
          ellipsoid.e[3 * x + y] = C(x, y);
        else
          ellipsoid.e[3 * x + y] = 0;
      }
    }
    ellipsoids_arr.ellipsoids.push_back(ellipsoid);
  }

  return ellipsoids_arr;
}

} // namespace sfc

#endif // _SFC__POLYTOPE_SFC_H_