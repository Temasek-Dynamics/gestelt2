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

#ifndef _VIZ_HELPER_HPP_
#define _VIZ_HELPER_HPP_

#include <Eigen/Eigen>

#include <rclcpp/rclcpp.hpp>

#include <visualization_msgs/msg/marker.hpp>

namespace viz_helper{

  /* Colors */
  static const Eigen::Vector4d plan_req_start_color{243/256.0, 156/256.0, 18/256.0, 0.5}; // Plan request start
  static const Eigen::Vector4d plan_req_rhp_goal_color{46.0/256.0, 134/256.0, 193/256.0, 0.5}; // Plan request RHP goal
  static const Eigen::Vector4d plan_req_goal_color{46.0/256.0, 204.0/256.0, 113.0/256.0, 0.5}; // Plan request goal

  static const Eigen::Vector4d fe_plan_start_color{1.0, 1.0, 0.0, 0.5}; // Front-end start
  static const Eigen::Vector4d fe_plan_goal_color{0.0, 1.0, 0.0, 0.5}; // Front-end goal
  static const Eigen::Vector4d fe_plan_path_color{1.0, 0.5, 0.0, 0.7}; // Front-end path
  static const  Eigen::Vector4d exec_traj_color{1, 51.0/221.0, 51.0/221.0, 0.5}; // #FF3333

  static const Eigen::Vector4d agent_id_text_color{1.0, 1.0, 0.0, 0.5}; // Agent ID text color

  static const double AGENT_ID_TEXT_SIZE{0.3};

  static const double EXEC_TRAJ_ALPHA{0.3};
  static const double EXEC_TRAJ_RADIUS{0.175};

  static const double FRONT_END_ALPHA{0.2};
  static const double FRONT_END_RADIUS{0.1};

  using namespace visualization_msgs::msg;

  struct VizHelper{

    template <typename T>
    using Publisher = typename rclcpp::Publisher<T>::SharedPtr;

    VizHelper(rclcpp::Clock::SharedPtr clock)
    : clock_(clock)
    {}

    /*Helper methods for publishing to RVIZ for visualization */

    void pubText(const Eigen::Vector3d& pos, Publisher<Marker>& publisher, const int& agent_id, const std::string& frame_id)
    {
      visualization_msgs::msg::Marker text;

      text.header.frame_id = frame_id;
      text.ns = "agent_id";
      text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      text.action = visualization_msgs::msg::Marker::ADD;
      text.id = 0;

      text.color.r = agent_id_text_color(0);
      text.color.g = agent_id_text_color(1);
      text.color.b = agent_id_text_color(2);
      text.color.a  = agent_id_text_color(3);

      text.text = "D"+std::to_string(agent_id);

      text.scale.z = AGENT_ID_TEXT_SIZE;

      text.pose.position.x = pos(0) + AGENT_ID_TEXT_SIZE;
      text.pose.position.y = pos(1) + AGENT_ID_TEXT_SIZE;
      text.pose.position.z = pos(2) + AGENT_ID_TEXT_SIZE;

      publisher->publish(text);
    }

    void pubPlanRequestViz(
      const Eigen::Vector3d& map_start, 
      const Eigen::Vector3d& rhp_goal, 
      const Eigen::Vector3d& map_goal, 
      Publisher<Marker>& publisher,
      const std::string& frame_id)
    {
      visualization_msgs::msg::Marker s_sph, rhp_sph, g_sph;
      double radius = 0.75;

      /* Start/goal sphere*/
      // s_sph.header.stamp = g_sph.header.stamp = clock_->now();
      s_sph.header.frame_id = g_sph.header.frame_id = rhp_sph.header.frame_id = frame_id;
      s_sph.ns = rhp_sph.ns = g_sph.ns = "plan_req_viz";
      s_sph.type = rhp_sph.type = g_sph.type = visualization_msgs::msg::Marker::CUBE;
      s_sph.action = rhp_sph.action = g_sph.action = visualization_msgs::msg::Marker::ADD;
      s_sph.id = 0;
      g_sph.id = 1; 
      rhp_sph.id = 2; 
      s_sph.pose.orientation.w = rhp_sph.pose.orientation.w = g_sph.pose.orientation.w = 1.0;

      s_sph.color.r = plan_req_start_color(0);
      s_sph.color.g = plan_req_start_color(1);
      s_sph.color.b = plan_req_start_color(2);
      s_sph.color.a  = plan_req_start_color(3);

      rhp_sph.color.r = plan_req_rhp_goal_color(0);
      rhp_sph.color.g = plan_req_rhp_goal_color(1);
      rhp_sph.color.b = plan_req_rhp_goal_color(2);
      rhp_sph.color.a = plan_req_rhp_goal_color(3);

      g_sph.color.r = plan_req_goal_color(0);
      g_sph.color.g = plan_req_goal_color(1);
      g_sph.color.b = plan_req_goal_color(2);
      g_sph.color.a = plan_req_goal_color(3);

      s_sph.scale.x = g_sph.scale.x = rhp_sph.scale.x =radius;
      s_sph.scale.y = g_sph.scale.y = rhp_sph.scale.y =radius;
      s_sph.scale.z = g_sph.scale.z = rhp_sph.scale.z =radius;

      /* Set Start */
      s_sph.pose.position.x = map_start(0);
      s_sph.pose.position.y = map_start(1);
      s_sph.pose.position.z = map_start(2);

      /* Set RHP Goal */
      rhp_sph.pose.position.x = rhp_goal(0);
      rhp_sph.pose.position.y = rhp_goal(1);
      rhp_sph.pose.position.z = rhp_goal(2);

      /* Set Goal */
      g_sph.pose.position.x = map_goal(0);
      g_sph.pose.position.y = map_goal(1);
      g_sph.pose.position.z = map_goal(2);

      publisher->publish(s_sph);
      publisher->publish(rhp_sph);
      publisher->publish(g_sph);
    }

    void pubSpaceTimePath(const std::vector<Eigen::Vector4d>& path,
                          Publisher<Marker>& publisher, 
                          const std::string& frame_id) 
    {
      visualization_msgs::msg::Marker cube;
      double radius = 0.1;
      double alpha = 0.8; 

      /* Fixed parameters */
      cube.header.frame_id = frame_id;
      // cube.header.stamp = clock_->now();
      cube.ns = "space_time_path";
      cube.type = visualization_msgs::msg::Marker::CUBE;
      cube.action = visualization_msgs::msg::Marker::ADD;
      cube.pose.orientation.w = 1.0;
      cube.color.a = alpha;

      // size
      cube.scale.x = radius;
      cube.scale.y = radius;
      cube.scale.z = radius;

      rclcpp::Rate loop_rate(200);

      for (size_t i = 0; i < path.size(); i++)
      {
        cube.id = i; 

        cube.pose.position.x = path[i](0);
        cube.pose.position.y = path[i](1);
        cube.pose.position.z = path[i](2);

        // Make the color value scale from 0.0 to 1.0 depending on the distance to the goal. 
        double time_ratio = std::clamp((path[i](3))/path.size(), 0.0, 1.0);

        cube.color.r = time_ratio ;
        cube.color.g = 0.0; 
        cube.color.b = 1.0 - time_ratio; 

        publisher->publish(cube);

        loop_rate.sleep();
      }
    }

    void pubFrontEndPath( const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>& path, 
                          Publisher<Marker>& publisher,
                          const std::string& frame_id) 
    {
      visualization_msgs::msg::Marker wps, lines;
      visualization_msgs::msg::Marker s_sph, g_sph;
      double radius = FRONT_END_RADIUS;
      double alpha = FRONT_END_ALPHA; 

      geometry_msgs::msg::Point pt;

      /* Start/goal sphere*/
      s_sph.header.frame_id = g_sph.header.frame_id = frame_id;
      // s_sph.header.stamp = g_sph.header.stamp = clock_->now();
      s_sph.ns = g_sph.ns = "fe_path_start_goal";
      s_sph.type = g_sph.type = visualization_msgs::msg::Marker::SPHERE;
      s_sph.action = g_sph.action = visualization_msgs::msg::Marker::ADD;
      s_sph.id = 0;
      g_sph.id = 1; 
      s_sph.pose.orientation.w = g_sph.pose.orientation.w = 1.0;

      s_sph.color.r = fe_plan_start_color(0); 
      s_sph.color.g = fe_plan_start_color(1); 
      s_sph.color.b = fe_plan_start_color(2); 
      s_sph.color.a = g_sph.color.a = alpha;

      g_sph.color.r = fe_plan_goal_color(0); 
      g_sph.color.g = fe_plan_goal_color(1); 
      g_sph.color.b = fe_plan_goal_color(2); 

      s_sph.scale.x = g_sph.scale.x = radius;
      s_sph.scale.y = g_sph.scale.y = radius;
      s_sph.scale.z = g_sph.scale.z = radius;

      /* wps: Sphere list (Waypoints) */
      wps.header.frame_id = frame_id;
      // wps.header.stamp = clock_->now();
      wps.ns = "fe_path_waypoints"; 
      wps.type = visualization_msgs::msg::Marker::SPHERE_LIST;
      wps.action = visualization_msgs::msg::Marker::ADD;
      wps.id = 0; 
      wps.pose.orientation.w = 1.0;

      wps.color.r = fe_plan_path_color(0);
      wps.color.g = fe_plan_path_color(1);
      wps.color.b = fe_plan_path_color(2);
      wps.color.a = alpha;

      wps.scale.x = wps.scale.y = wps.scale.z = radius;

      /* lines: Line strips (Connecting waypoints) */
      lines.header.frame_id = frame_id;
      // lines.header.stamp = clock_->now();
      lines.ns = "fe_path_lines"; 
      lines.type = visualization_msgs::msg::Marker::LINE_STRIP;
      lines.action = visualization_msgs::msg::Marker::ADD;
      lines.id = 0;
      lines.pose.orientation.x = lines.pose.orientation.y = lines.pose.orientation.z = 0.0;
      lines.pose.orientation.w = 1.0;

      lines.color.r = fe_plan_path_color(0);
      lines.color.g = fe_plan_path_color(1);
      lines.color.b = fe_plan_path_color(2);
      lines.color.a = alpha * 0.75;

      lines.scale.x = radius * 0.5;

      s_sph.pose.position.x = pt.x = path[0](0);
      s_sph.pose.position.y = pt.y = path[0](1);
      s_sph.pose.position.z = pt.z = path[0](2);

      lines.points.push_back(pt);

      for (size_t i = 1; i < path.size()-1; i++){
        pt.x = path[i](0);
        pt.y = path[i](1);
        pt.z = path[i](2);

        wps.points.push_back(pt);
        lines.points.push_back(pt);
      }

      g_sph.pose.position.x = pt.x = path.back()(0);
      g_sph.pose.position.y = pt.y = path.back()(1);
      g_sph.pose.position.z = pt.z = path.back()(2);

      lines.points.push_back(pt);

      publisher->publish(s_sph);
      publisher->publish(g_sph);
      publisher->publish(wps);
      publisher->publish(lines);
    }

    void pubVoroVertices( const std::vector<Eigen::Vector3d>& vor_verts,  
                          Publisher<Marker>& publisher,
                          const std::string& frame_id)
    {
      visualization_msgs::msg::Marker vertices;
      double radius = 0.2;
      double alpha = 0.8; 

      /* vertices: Sphere list (voronoi graph vertices) */
      vertices.header.frame_id = frame_id;
      vertices.header.stamp = clock_->now();
      vertices.ns = "voronoi_vertices"; 
      vertices.type = visualization_msgs::msg::Marker::SPHERE_LIST;
      vertices.action = visualization_msgs::msg::Marker::ADD;
      vertices.id = 1; 
      vertices.pose.orientation.w = 1.0;

      vertices.color.r = 0.0;
      vertices.color.g = 0.5;
      vertices.color.b = 1.0;
      vertices.color.a = alpha;

      vertices.scale.x = radius;
      vertices.scale.y = radius;
      vertices.scale.z = radius;

      geometry_msgs::msg::Point pt;
      for (size_t i = 0; i < vor_verts.size(); i++){
        pt.x = vor_verts[i](0);
        pt.y = vor_verts[i](1);
        pt.z = vor_verts[i](2);

        vertices.points.push_back(pt);
      }

      publisher->publish(vertices);
    }

    void pubFrontEndClosedList( const std::vector<Eigen::Vector3d>& pts, 
                                Publisher<Marker>& publisher, 
                                const std::string& frame_id)
    {
      if (pts.empty()){
        return;
      }

      Eigen::Vector3d color = Eigen::Vector3d{0.0, 0.0, 0.0};
      double radius = 0.1;
      double alpha = 0.7;

      visualization_msgs::msg::Marker sphere_list;

      sphere_list.header.frame_id = frame_id;
      // sphere_list.header.stamp = clock_->now();
      sphere_list.type = visualization_msgs::msg::Marker::SPHERE_LIST;
      sphere_list.action = visualization_msgs::msg::Marker::ADD;
      sphere_list.ns = "closed_list"; 
      sphere_list.id = 0; 
      sphere_list.pose.orientation.w = 1.0;

      sphere_list.color.r = color(0);
      sphere_list.color.g = color(1);
      sphere_list.color.b = color(2);
      sphere_list.color.a = alpha;

      sphere_list.scale.x = radius;
      sphere_list.scale.y = radius;
      sphere_list.scale.z = radius;

      geometry_msgs::msg::Point pt;
      for (size_t i = 0; i < pts.size(); i++){
        pt.x = pts[i](0);
        pt.y = pts[i](1);
        pt.z = pts[i](2);

        sphere_list.points.push_back(pt);
      }

      publisher->publish(sphere_list);
    }

    /* Publish a set of sampled trajectory points*/
    static void pubExecTraj(const Eigen::MatrixXd& traj_pts, 
                            Publisher<Marker>& publisher,
                            const std::string& frame_id)
    {
      visualization_msgs::msg::Marker wps, lines;
      double radius = EXEC_TRAJ_RADIUS;
      double alpha = EXEC_TRAJ_ALPHA; 

      geometry_msgs::msg::Point pt;

      /* wps: Sphere list (Waypoints) */
      wps.header.frame_id = lines.header.frame_id =  frame_id;
      // wps.header.stamp = clock_->now();
      wps.ns = "exec_traj_waypoints"; 
      wps.type = visualization_msgs::msg::Marker::SPHERE_LIST;
      wps.action = visualization_msgs::msg::Marker::ADD;
      wps.id = 0; 
      wps.pose.orientation.x = wps.pose.orientation.y = wps.pose.orientation.z = 0.0;
      wps.pose.orientation.w = 1.0;

      wps.color.r = exec_traj_color(0);
      wps.color.g = exec_traj_color(1);
      wps.color.b = exec_traj_color(2);
      wps.color.a = alpha;

      wps.scale.x = wps.scale.y = wps.scale.z = radius;

      /* lines: Line strips (Connecting waypoints) */
      // lines.header.stamp = clock_->now();
      lines.ns = "exec_traj_lines"; 
      lines.type = visualization_msgs::msg::Marker::LINE_STRIP;
      lines.action = visualization_msgs::msg::Marker::ADD;
      lines.id = 0;
      lines.pose.orientation.x = lines.pose.orientation.y = lines.pose.orientation.z = 0.0;
      lines.pose.orientation.w = 1.0;

      lines.color.r = exec_traj_color(0);
      lines.color.g = exec_traj_color(1);
      lines.color.b = exec_traj_color(2);
      lines.color.a = alpha * 0.5;

      lines.scale.x = radius * 0.5;

      pt.x = traj_pts.col(0)(0);
      pt.y = traj_pts.col(0)(1);
      pt.z = traj_pts.col(0)(2);
      lines.points.push_back(pt);

      for (int i = 1; i < traj_pts.cols() - 1; i++)
      {
        pt.x = traj_pts.col(i)(0);
        pt.y = traj_pts.col(i)(1);
        pt.z = traj_pts.col(i)(2);

        wps.points.push_back(pt);
        lines.points.push_back(pt);
      }

      pt.x = traj_pts.rightCols(1)(0);
      pt.y = traj_pts.rightCols(1)(1);
      pt.z = traj_pts.rightCols(1)(2);
      lines.points.push_back(pt);

      publisher->publish(wps);
      publisher->publish(lines);
    }

    /* Members */

    rclcpp::Clock::SharedPtr clock_;

  }; // struct VizHelper

} // namespace viz_helper

#endif // _VIZ_HELPER_HPP_
