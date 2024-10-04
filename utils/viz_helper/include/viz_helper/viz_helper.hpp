#ifndef _VIZ_HELPER_HPP_
#define _VIZ_HELPER_HPP_

#include <Eigen/Eigen>

#include <rclcpp/rclcpp.hpp>

#include <visualization_msgs/msg/marker.hpp>

namespace viz_helper{

  /* Colors */
  static const Eigen::Vector4d fe_plan_start_color{1.0, 1.0, 0.0, 0.5}; // Front-end start
  static const Eigen::Vector4d fe_plan_goal_color{0.0, 1.0, 0.0, 0.5}; // Front-end goal
  static const Eigen::Vector4d fe_plan_path_color{1.0, 0.5, 0.0, 0.5}; // Front-end path
  static const  Eigen::Vector4d exec_traj_color{1, 51.0/221.0, 51.0/221.0, 0.5}; // #FF3333

  static const double EXEC_TRAJ_ALPHA{0.2};
  static const double EXEC_TRAJ_RADIUS{0.1};

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

    void pubStartGoalPts(
      const Eigen::Vector3d& map_start, 
      const Eigen::Vector3d& map_goal, 
      Publisher<Marker>& publisher,
      const std::string& frame_id)
    {
      visualization_msgs::msg::Marker start_sphere, goal_sphere;
      double radius = 0.75;
      double alpha = 0.5; 

      /* Start/goal sphere*/
      start_sphere.header.frame_id = goal_sphere.header.frame_id = frame_id;
      // start_sphere.header.stamp = goal_sphere.header.stamp = clock_->now();
      start_sphere.ns = goal_sphere.ns = "plan_req_viz";
      start_sphere.type = goal_sphere.type = visualization_msgs::msg::Marker::CUBE;
      start_sphere.action = goal_sphere.action = visualization_msgs::msg::Marker::ADD;
      start_sphere.id = 0;
      goal_sphere.id = 1; 
      start_sphere.pose.orientation.w = goal_sphere.pose.orientation.w = 1.0;

      start_sphere.color.r = 1.0; 
      start_sphere.color.g = 1.0; 
      start_sphere.color.b = 0.0; 
      start_sphere.color.a = goal_sphere.color.a = alpha;

      goal_sphere.color.r = 0.0;
      goal_sphere.color.g = 1.0;
      goal_sphere.color.b = 0.0;

      start_sphere.scale.x = goal_sphere.scale.x = radius;
      start_sphere.scale.y = goal_sphere.scale.y = radius;
      start_sphere.scale.z = goal_sphere.scale.z = radius;

      /* Set Start */
      start_sphere.pose.position.x = map_start(0);
      start_sphere.pose.position.y = map_start(1);
      start_sphere.pose.position.z = map_start(2);

      /* Set Goal */
      goal_sphere.pose.position.x = map_goal(0);
      goal_sphere.pose.position.y = map_goal(1);
      goal_sphere.pose.position.z = map_goal(2);

      publisher->publish(start_sphere);
      publisher->publish(goal_sphere);
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

    void pubFrontEndPath( const std::vector<Eigen::Vector3d>& path, 
                          Publisher<Marker>& publisher,
                          const std::string& frame_id) 
    {
      visualization_msgs::msg::Marker wps, lines;
      visualization_msgs::msg::Marker start_sphere, goal_sphere;
      double radius = FRONT_END_RADIUS;
      double alpha = FRONT_END_ALPHA; 

      geometry_msgs::msg::Point pt;

      /* Start/goal sphere*/
      start_sphere.header.frame_id = goal_sphere.header.frame_id = frame_id;
      // start_sphere.header.stamp = goal_sphere.header.stamp = clock_->now();
      start_sphere.ns = goal_sphere.ns = "fe_path_start_goal";
      start_sphere.type = goal_sphere.type = visualization_msgs::msg::Marker::SPHERE;
      start_sphere.action = goal_sphere.action = visualization_msgs::msg::Marker::ADD;
      start_sphere.id = 0;
      goal_sphere.id = 1; 
      start_sphere.pose.orientation.w = goal_sphere.pose.orientation.w = 1.0;

      start_sphere.color.r = fe_plan_start_color(0); 
      start_sphere.color.g = fe_plan_start_color(1); 
      start_sphere.color.b = fe_plan_start_color(2); 
      start_sphere.color.a = goal_sphere.color.a = alpha;

      goal_sphere.color.r = fe_plan_goal_color(0); 
      goal_sphere.color.g = fe_plan_goal_color(1); 
      goal_sphere.color.b = fe_plan_goal_color(2); 

      start_sphere.scale.x = goal_sphere.scale.x = radius;
      start_sphere.scale.y = goal_sphere.scale.y = radius;
      start_sphere.scale.z = goal_sphere.scale.z = radius;

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

      start_sphere.pose.position.x = pt.x = path[0](0);
      start_sphere.pose.position.y = pt.y = path[0](1);
      start_sphere.pose.position.z = pt.z = path[0](2);

      lines.points.push_back(pt);

      for (size_t i = 1; i < path.size()-1; i++){
        pt.x = path[i](0);
        pt.y = path[i](1);
        pt.z = path[i](2);

        wps.points.push_back(pt);
        lines.points.push_back(pt);
      }

      goal_sphere.pose.position.x = pt.x = path.back()(0);
      goal_sphere.pose.position.y = pt.y = path.back()(1);
      goal_sphere.pose.position.z = pt.z = path.back()(2);

      lines.points.push_back(pt);

      publisher->publish(start_sphere);
      publisher->publish(goal_sphere);
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
      wps.header.frame_id = frame_id;
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
      lines.header.frame_id = frame_id;
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
      lines.color.a = alpha * 0.75;

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
