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

#ifndef OCC_MAP__OCC_MAP_HPP_
#define OCC_MAP__OCC_MAP_HPP_

#include <Eigen/Eigen>

#include <rclcpp/rclcpp.hpp>

#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/node_utils.hpp"

#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>

#include <std_msgs/msg/empty.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <geometry_msgs/msg/point32.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2/exceptions.h>

#include <ikd_tree/ikd_tree.hpp>

#include <bonxai/bonxai.hpp>
#include <bonxai/pcl_utils.hpp>
#include <bonxai/probabilistic_map.hpp>


#include <logger_wrapper/logger_wrapper.hpp>
#include <logger_wrapper/timer.hpp>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace std::placeholders;

namespace occ_map
{
  /* Fixed constants used during mapping*/
  struct MappingParameters
  {
    // Local and global map are bounded 3d boxes
    Eigen::Vector3d global_map_origin_; // Origin of map (Set to be the corner of the map)
    Eigen::Vector3d local_map_origin_; // Origin of local map (Set to be the corner of the map) w.r.t map frame
    Eigen::Vector3d local_map_max_; // max position of local map (Set to be the corner of the map)
    
    Eigen::Vector3d global_map_size_; //  Size of global occupancy map  (m)
    Eigen::Vector3d local_map_size_; //  Size of local occupancy map (m)

    Eigen::Matrix4d world_to_map{Eigen::Matrix4d::Identity(4, 4)};
    Eigen::Matrix4d map_to_world{Eigen::Matrix4d::Identity(4, 4)};

    Eigen::Vector3i local_map_num_voxels_; //  Size of local occupancy grid (no. of voxels)

    double resolution_;   // Also defined as the size of each individual voxel                 
    double inv_resolution_;   // Also defined as the size of each individual voxel                 
    double agent_inflation_;    // [Comm-less] Dynamic obstacle Inflation in units of meters
    double static_inflation_;    // Static obstacle inflation in units of meters
    int inf_static_vox_;  // Inflation in number of voxels, = inflation_/resolution_ 
    int inf_dyn_vox_; // Inflation in number of voxels

    double pose_timeout_; // Timeout for pose update before emergency stop is activated
    double max_range; // Max sensor range

    /* visualization and computation time display */
    std::string global_frame; // frame id of global map reference 
    std::string map_frame; // frame id of UAV origin 
    std::string camera_frame; // frame id of camera link
    std::string base_link_frame; // frame id of base_link
  };

  /* Dynamic data used during mapping */
  struct MappingData
  {
    // [DYNAMIC]: Homogenous Transformation matrix of camera to fixed map frame
    Eigen::Matrix4d cam_to_map{Eigen::Matrix4d::Identity(4, 4)};

    // Inverse of cam_to_map
    Eigen::Matrix4d map_to_cam{Eigen::Matrix4d::Identity(4, 4)};

    // [DYNAMIC]: Homogenous Transformation matrix of fbase_link to map frame
    Eigen::Matrix4d bl_to_map{Eigen::Matrix4d::Identity(4, 4)};

    double last_cloud_cb_time{-1.0}; // True if cloud and odom has timed out

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

class OccMap : public nav2_util::LifecycleNode
{
public:

  /** Initialization methods */
  /**
   * @brief  Constructor for the wrapper
   * @param options Additional options to control creation of the node.
   */
  explicit OccMap(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  explicit OccMap(
    const std::string & name,
    const std::string & parent_namespace,
    const bool & use_sim_time);

  ~OccMap();

  // Get occupancy grid resolution
  inline double getRes() const{
    return mp_.resolution_;
  }

  // Get global map origin (This is defined to be a corner of the global map i.e. (-W/2, -L/2, 0))
  inline Eigen::Vector3d getGlobalOrigin() const{
    return mp_.global_map_origin_; 
  }

  // Get local map origin (This is defined to be a corner of the local map i.e. (-local_W/2, -local_L/2, 0))
  inline Eigen::Vector3d getLocalMapOrigin() const{
    return mp_.local_map_origin_;
  }

  inline Eigen::Vector3d getLocalMapMax() const{
    return mp_.local_map_max_;
  }

  inline Eigen::Vector3d getLocalMapOrigin(const double& offset = 0.0) const{
    return mp_.local_map_origin_ + Eigen::Vector3d::Constant(offset);
  }

  inline Eigen::Vector3d getLocalMapMax(const double& offset = 0.0) const{
    return mp_.local_map_max_ - Eigen::Vector3d::Constant(offset); 
  }

  // Get points in local map (in fixed map frame). Used by safe flight corridor generation
  inline std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> getLclObsPts(){
    std::lock_guard<std::mutex> lcl_occ_map_guard(lcl_occ_map_mtx_);
    return lcl_pts_in_global_frame_;
  }

  // Takes in position [global_frame] and check if within global map
  inline bool inGlobalMap(const Eigen::Vector3d &pos){
    return (pos(0) >= -mp_.global_map_size_(0)/2 && pos(0) < mp_.global_map_size_(0)/2
      && pos(1) >= -mp_.global_map_size_(1)/2 && pos(1) < mp_.global_map_size_(1)/2
      && pos(2) >= 0.0 && pos(2) < mp_.global_map_size_(2));
  }

  // Takes in position [global_frame] and check if within global map
  inline bool inGlobalMapIdx(const Eigen::Vector3i &idx){
    const auto pos = idxToPos(idx);

    return (pos(0) >= -mp_.global_map_size_(0)/2 && pos(0) < mp_.global_map_size_(0)/2
      && pos(1) >= -mp_.global_map_size_(1)/2 && pos(1) < mp_.global_map_size_(1)/2
      && pos(2) >= 0.0 && pos(2) < mp_.global_map_size_(2));
  }

  inline bool inLocalMap(const Eigen::Vector3d &pos){
    return (pos(0) >= mp_.local_map_origin_(0)   && pos(0) < mp_.local_map_max_(0)
    && pos(1) >= mp_.local_map_origin_(1)  && pos(1) < mp_.local_map_max_(1)
    && pos(2) >= mp_.local_map_origin_(2)  && pos(2) < mp_.local_map_max_(2));
  }

  inline Eigen::Vector3i posToIdx(const Eigen::Vector3d &pos){
    return Eigen::Vector3i{ int32_t(pos(0) * mp_.inv_resolution_) - std::signbit(pos(0)),
                            int32_t(pos(1) * mp_.inv_resolution_) - std::signbit(pos(1)),
                            int32_t(pos(2) * mp_.inv_resolution_) - std::signbit(pos(2)) };
  }

  inline Eigen::Vector3d idxToPos(const Eigen::Vector3i &idx){
    return Eigen::Vector3d{(static_cast<double>(idx(0)) + 0.5) * mp_.resolution_,
                            (static_cast<double>(idx(1)) + 0.5) * mp_.resolution_,
                            (static_cast<double>(idx(2)) + 0.5) * mp_.resolution_};
  }

  void mapToWorld(const Eigen::Vector3d& pos_map, Eigen::Vector3d& pos_world){
    pos_world = (mp_.map_to_world * pos_map.homogeneous()).hnormalized(); 
  }

  void worldToMap(const Eigen::Vector3d& pos_world, Eigen::Vector3d& pos_map){
    pos_map = (mp_.world_to_map * pos_world.homogeneous()).hnormalized();  
  }

  // Takes in position in [global_frame] and check if occupied
  int getCost(const Eigen::Vector3d &pos);

  int getCostIdx(const Eigen::Vector3i &idx);
  
  protected:

  /**
   * @brief Configure node
   */
  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Activate node
   */
  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Deactivate node
   */
  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Cleanup node
   */
  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief shutdown node
   */
  nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief as a child-LifecycleNode :
   * OccMap may be launched by another Lifecycle Node as a composed module
   * If composed, its parents will handle the shutdown, which includes this module
   */
  void on_rcl_preshutdown() override
  {
    if (is_lifecycle_follower_) {
      // Transitioning handled by parent node
      return;
    }

    // Else, if this is an independent node, this node needs to handle itself.
    RCLCPP_INFO(
      get_logger(), "Running Nav2 LifecycleNode rcl preshutdown (%s)",
      this->get_name());

    runCleanups();

    destroyBond();
  }


private:

  void init();

  void getParameters();

  /* Core methods */
  
  // Called by planners to update the local map
  void updateLocalMap();

  /**
   * @brief Publish map for visualization
   * 
   */
  void publishOccMap(const pcl::PointCloud<pcl::PointXYZ>::Ptr& pcd);

  // Publish local map bounds
  void publishLocalMapBounds();

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /*Subscriber Callbacks*/
  void resetMapCB(const std_msgs::msg::Empty::SharedPtr );

  // Subscriber callback to point cloud 
  void cloudCB(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  rcl_interfaces::msg::SetParametersResult dynamicParametersCB(
    const std::vector<rclcpp::Parameter> & parameters);

  /*Timer Callbacks*/

  /**
   * @brief This timer publishes a visualization of the occupancy grid
  */
  void vizMapTimerCB();

  /**
   * @brief This timer updates the local map for use by planners
  */
  void updateLocalMapTimerCB();

  // Checks if time elapsed has exceeded a given threshold
  inline bool isTimeout(const double& last_state_time, const double& threshold)
  {
    return (get_clock()->now().seconds() - last_state_time) >= threshold;
  } 

  bool is_lifecycle_follower_{true};   ///< whether is a child-LifecycleNode or an independent node

  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
  std::unique_ptr<nav2_util::NodeThread> executor_thread_;

	/* Callback groups */
	rclcpp::CallbackGroup::SharedPtr callback_group_;

  std::string name_;
  std::string parent_namespace_;

  /* Params */
  int drone_id_{0}; //Drone ID

  bool initial_transform_timeout_{false};
  bool transform_tolerance_{false};

  // Noise filter
  double noise_search_radius_{0.3}; // Radius to search for neighbouring points
  int noise_min_neighbors_{1}; // Minimum number of neighbors within 'search_radius' to be a valid point

  // Passthrough filter for point cloud 
  double cloud_in_min_z_{0.0};
  double cloud_in_max_z_{10.0};

  bool print_timer_{false}; // Flag to enable printing of debug information such as timers

  double time_vel_{0.1}; // [s] time along velocity vector to mark as occupied

  double viz_occ_map_freq_{-1.0}; // Frequency to publish occupancy map visualization
  double update_local_map_freq_{-1.0};  // Frequency to update local map

  MappingParameters mp_;  // Parameters used for map

  Bonxai::ProbabilisticMap::Options bonxai_options_; // Bonxai probabilistic map options

  /* Params callback */
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;

  /* Subscribers */
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;

  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr reset_map_sub_;
  
  /* Publishers  */
	rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr occ_map_pub_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PolygonStamped>::SharedPtr local_map_bounds_pub_; // Publisher to show local map bounds

  /* Timers */
	rclcpp::TimerBase::SharedPtr viz_map_timer_;	 // Timer for visualizing map
	rclcpp::TimerBase::SharedPtr update_local_map_timer_; // Timer for updating local map

  // TF transformation 
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};

  /* Data structures for maps */
  MappingData md_;  // Mapping data

  std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> occ_pcd_in_lcl_frame_; // [LOCAL MAP FRAME] Occupancy map points formed by Bonxai probabilistic mapping (w.r.t local map origin)
  std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> occ_pcd_in_gbl_frame_; // [MAP FRAME] Occupancy map points formed by Bonxai probabilistic mapping (w.r.t local map origin)
  
  std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> raw_lcl_pcd_in_global_frame_; // Raw point clouds

  std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> lcl_pts_in_global_frame_; // Vector of obstacle points used for sfc generation

  std::shared_ptr<Bonxai::ProbabilisticMap> bonxai_map_; // Bonxai data structure 
  std::unique_ptr<KD_TREE<pcl::PointXYZ>> kdtree_; // KD-Tree 

  /* Flags */
  bool local_map_updated_{false}; // Indicates if first local map update is done 

  /* Mutexes */
  std::mutex bonxai_map_mtx_;  // Mutex lock for bonxai map
  std::mutex lcl_occ_map_mtx_;  // Mutex lock for occ_pcd_in_lcl_frame_

  /* Stopwatch for profiling performance */
  // logger_wrapper::Timer tm_update_local_map_{"OccMap::updateLocalMap"};  // Time required for map construction
  // logger_wrapper::Timer tm_bonxai_insert_{"bonxai::insertPointCloud"};  // Time required for map construction

  /* Point cloud Filters */
  pcl::PassThrough<pcl::PointXYZ> z_filter_cloud_in_;

  /* Logging */
	std::shared_ptr<logger_wrapper::LoggerWrapper> logger_;
};

// inline bool OccMap::getNearestOccupiedCell(const Eigen::Vector3d &pos, 
//                             Eigen::Vector3d& occ_nearest, double& dist_to_nearest_nb){
//   int nearest_num_nb = 1;
//   pcl::PointXYZ search_point(pos(0), pos(1), pos(2));
//   std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ>> nb_points;
//   std::vector<float> nb_radius_vec;

//   kdtree_->Nearest_Search(search_point, nearest_num_nb, nb_points, nb_radius_vec);

//   if (nb_points.empty()){
//     return false;
//   }

//   dist_to_nearest_nb = sqrt(nb_radius_vec[0]);

//   occ_nearest = Eigen::Vector3d{nb_points[0].x, nb_points[0].y, nb_points[0].z};

//   return true;
// }


}  // namespace occ_map

#endif  // OCC_MAP__OCC_MAP_HPP_
