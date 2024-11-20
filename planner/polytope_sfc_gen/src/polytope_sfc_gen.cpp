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

#include <polytope_sfc_gen/polytope_sfc_gen.hpp>

namespace sfc
{

PolytopeSFC::PolytopeSFC(const PolytopeSFCParams& sfc_params):
  params_(sfc_params)
{}   

void PolytopeSFC::reset()
{
    poly_vec_.clear();
    poly_constr_vec_.clear();
}

bool PolytopeSFC::generateSFC(const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> &obs_pts,
                              const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> &path_3d)
{
    EllipsoidDecomp3D ellip_decomp_util_; // Decomposition util for Liu's method
    
    //Using ellipsoid decomposition
    ellip_decomp_util_.set_obs(obs_pts);
    ellip_decomp_util_.set_local_bbox(Vec3f(params_.bbox_x, params_.bbox_y, params_.bbox_z)); 
    ellip_decomp_util_.dilate(path_3d); // Set max iteration number of 10, do fix the path

    //Publish visualization msgs
    // decomp_ros_msgs::msg::EllipsoidArray es_msg = ellipsoid_array_to_ros(ellip_decomp_util_.get_ellipsoids());
    // es_msg.header.frame_id = params_.map_frame;

    poly_vec_ = ellip_decomp_util_.get_polyhedrons();

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

decomp_ros_msgs::msg::PolyhedronArray PolytopeSFC::getSFCMsg()
{
  decomp_ros_msgs::msg::PolyhedronArray poly_msg = 
    polyhedron_array_to_ros(poly_vec_);
  poly_msg.header.frame_id = params_.map_frame;

  return poly_msg;
}

} // namespace sfc

