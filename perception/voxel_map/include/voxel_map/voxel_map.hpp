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

#ifndef VOXEL_MAP__VOXEL_MAP_HPP_
#define VOXEL_MAP__VOXEL_MAP_HPP_

#include "voxel_map/visibility_control.h"

#include <Eigen/Eigen>

#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>

// #include <ikd_tree/ikd_tree.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/wait_for_message.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/empty.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <geometry_msgs/msg/point32.hpp>

/* Debugging */
#include <visualization_msgs/msg/marker.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2/exceptions.h>

#include <bonxai/bonxai.hpp>
#include <bonxai/pcl_utils.hpp>
#include <bonxai/probabilistic_map.hpp>

#include <logger_wrapper/logger_wrapper.hpp>
#include <logger_wrapper/timer.hpp>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace std::placeholders;

namespace voxel_map
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

    Eigen::Vector3i local_map_num_voxels_; //  Size of local occupancy grid (no. of voxels)

    double resolution_;   // Also defined as the size of each individual voxel                 
    double agent_inflation_;    // [Comm-less] Dynamic obstacle Inflation in units of meters
    double static_inflation_;    // Static obstacle inflation in units of meters
    int inf_static_vox_;  // Inflation in number of voxels, = inflation_/resolution_ 
    int inf_dyn_vox_; // Inflation in number of voxels

    double pose_timeout_; // Timeout for pose update before emergency stop is activated
    double max_range; // Max sensor range

    /* visualization and computation time display */
    double ground_height_; // Lowest possible height (z-axis)

    std::string global_map_frame; // frame id of global map reference 
    std::string local_map_frame; // frame id of UAV origin 
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

  /* Boolean map structure used by planner*/
  struct BoolMap3D {  
    int z_sep_cm{-1};   // [cm] Separation between slice layers
    int min_height_cm{-1};     // [cm] Lowest slice height
    int max_height_cm{-1};     // [cm] Highest slice height

    double z_sep_m{-1.0}; // [m] separation between slice layers
    double min_height_m{-1.0};   // [m] Lowest slice height
    double max_height_m{-1.0};   // [m] Highest slice height

    Eigen::Vector3d origin{0.0, 0.0, 0.0}; // [m] Origin of local map

    int width{-1};            // [] Number of width cells 
    int height{-1};           // [] Number of height cells
    double resolution{-1.0};    // [cm] Resolution of map

    std::map<int, std::vector<bool>> bool_maps; // Map of BoolMap objects
  };

class VoxelMap
{
public: 
  // Custom type definition for message filters
  using SyncPolicyCloudOdom = message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, nav_msgs::msg::Odometry>;
  using SynchronizerCloudOdom =  std::shared_ptr<message_filters::Synchronizer<SyncPolicyCloudOdom>>;

  // Custom type definition for Bonxai
  using BonxaiT = Bonxai::ProbabilisticMap;

public:

  /** Initialization methods */

  /**
   * @brief Construct a new Voxel Map object
   * 
   * @param node 
   * @param map_origin Only used for debugging with full point cloud map
   * @param num_drones 
   */
  VoxelMap(rclcpp::Node::SharedPtr node, 
            const Eigen::Vector3d& map_origin,
            const int& num_drones);

  virtual ~VoxelMap();

  // Reset map data
  void reset(const double& resolution);

  void init();

  void initParams();

  void initPubSubTimer();

  /* Core methods */
  
  // Convert point cloud message to point cloud map, transform it from camera-to-global frame and save it. 
  void pcd2MsgToMap(const sensor_msgs::msg::PointCloud2& msg);
  
  // Called by planners to update the local map
  void updateLocalMap();

  /**
   * @brief Get all point clouds at a slice of the local map centered at height z with specified thickness
   * 
   * @param slice_z [cm] Slice at height z
   * @param thickness [m] Thickness of the map
   * @param bool_map 1D boolean map
   * @return boolean map array
   */
  void getMapSlice(const double& slice_z_cm, const double& thickness, std::vector<bool>& bool_map);

  /** Publisher methods */

  /**
   * @brief Publish map for visualization
   * 
   */
  void publishOccMap(const pcl::PointCloud<pcl::PointXYZ>::Ptr& pcd);

  /**
   * @brief Publish a z slice of the map for visualization
   * 
   */
  void publishSliceMap(const pcl::PointCloud<pcl::PointXYZ>::Ptr& slice_map);

  // Publish sphere to indicate collision with interpolated colors between fatal and warning radius
  void publishCollisionSphere(
    const Eigen::Vector3d &pos, const double& dist_to_obs, 
    const double& fatal_radius, const double& warn_radius);

  // Publish local map bounds
  void publishLocalMapBounds();

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /*Subscriber Callbacks*/

  void resetMapCB(const std_msgs::msg::Empty::SharedPtr );

  // Subscriber callback to point cloud and odom
  void cloudCB(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  /*Timer Callbacks*/

  /**
   * @brief This timer publishes a visualization of the occupancy grid
  */
  void vizMapTimerCB();

  /**
   * @brief This timer updates the local map for use by planners
  */
  void updateLocalMapTimerCB();

  /**
   * @brief This timer publishes the map to local map TF
  */
  void pubLocalMapTFTimerCB();


  /**
   * @brief Timer for checking collision of drone with obstacles
   * 
   */
  void checkCollisionsTimerCB();

public:

  /* Setter methods */
  void updateSwarmState(
    const int& id,
    Eigen::Vector3d& swarm_pose, 
    Eigen::Vector3d& swarm_vel);

  /* Getter methods */

  // Get occupancy grid resolution
  double getRes() const;

  // Get global map origin (This is defined to be a corner of the global map i.e. (-W/2, -L/2, 0))
  Eigen::Vector3d getGlobalOrigin() const;

  // Get local map origin (This is defined to be a corner of the local map i.e. (-local_W/2, -local_L/2, 0))
  Eigen::Vector3d getLocalMapOrigin() const;

  Eigen::Vector3d getLocalMapMax() const;

  Eigen::Vector3d getLocalMapOrigin(const double& offset) const;
  Eigen::Vector3d getLocalMapMax(const double& offset) const;


  // Get points in local map (in fixed map frame). Used by safe flight corridor generation
  std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> getLclObsPts();

  // Get inflation value
  double getInflation() const;

  bool getBoolMap3D(BoolMap3D& bool_map_3d);

  // Takes in position in [global_map_frame] and check if occupied
  bool isOccupied(const Eigen::Vector3d &pos);

/* Checks */
private: 

  // Checks if time elapsed has exceeded a given threshold
  bool isTimeout(const double& last_state_time, const double& threshold);


  // Takes in position [global_map_frame] and check if within global map
  bool isInGlobalMap(const Eigen::Vector3d &pos);

  // Takes in position [global_map_frame] and check if within local map
  bool isInLocalMap(const Eigen::Vector3d &pos);


  /**
   * @brief Get the Nearest Occupied Cell  
   * 
   * @param pos 
   * @param occ_nearest position of nearest occupied cell
   * @param radius 
   * @return true 
   * @return false 
   */
  // bool getNearestOccupiedCell(const Eigen::Vector3d &pos, 
  //                             Eigen::Vector3d& occ_nearest, double& dist_to_nearest_nb);

private: 
  rclcpp::Node::SharedPtr node_;

	/* Callback groups */
	rclcpp::CallbackGroup::SharedPtr mapping_group_;
	rclcpp::CallbackGroup::SharedPtr tf_broadcast_group_;

  /* Params */
  int drone_id_{0}; //Drone ID

  double map_slicing_sample_thickness_; // [m] map slice sampling thickness

  bool verbose_print_{false}; // Flag to enable printing of debug information such as timers
  bool dbg_input_entire_map_{false}; // flag to indicate that map will be constructed at the start from the entire pcd map (instead of through incremental sensor data)
  std::string entire_pcd_map_topic_; // Topic to listen for an entire PCD for debugging

  bool dyn_obs_mark_in_occ_map_{false}; // flag to mark dynamic obstacles in occupancy map as occupied
  double time_vel_{0.1}; // [s] time along velocity vector to mark as occupied

  bool check_collisions_{true}; // Flag for checking collisions
  double col_warn_radius_, col_fatal_radius_; // collision check radius

  double viz_occ_map_freq_{-1.0}; // Frequency to publish occupancy map visualization
  double update_local_map_freq_{-1.0};  // Frequency to update local map
  double check_col_freq_{-1.0};  // Frequency to update local map

  MappingParameters mp_;  // Parameters used for map

  /* Subscribers */

  SynchronizerCloudOdom sync_cloud_odom_; // Synchronization policy for cloud and odom topic
  
  // std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>> cloud_sub_;
  // std::shared_ptr<message_filters::Subscriber<nav_msgs::msg::Odometry>> odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;

  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr reset_map_sub_;
  
  /* Publishers  */
	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr occ_map_pub_;
  
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr slice_map_pub_; // Publisher for z slice of map
  rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr local_map_bounds_pub_; // Publisher to show local map bounds
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr collision_viz_pub_; // Publisher for collision visualization spheres

  /* Timers */
	rclcpp::TimerBase::SharedPtr viz_map_timer_;	 // Timer for visualizing map
	rclcpp::TimerBase::SharedPtr check_collisions_timer_; // Timer for checking collisions
	rclcpp::TimerBase::SharedPtr update_local_map_timer_; // Timer for updating local map
	rclcpp::TimerBase::SharedPtr pub_lcl_map_tf_timer_; // Timer for broadcasting map to local_map tf

  // TF transformation 
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_; // broadcast tf link from global map frame to local map origin 
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};

  /* Data structures for maps */
  MappingData md_;  // Mapping data

  std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> lcl_pcd_lclmapframe_; // [LOCAL MAP FRAME] Occupancy map points formed by Bonxai probabilistic mapping (w.r.t local map origin)
  std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> lcl_pcd_fixedmapframe_; // [MAP FRAME] Occupancy map points formed by Bonxai probabilistic mapping (w.r.t local map origin)
  std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> pcd_in_map_frame_;  // Point cloud global map in UAV Origin frame
  std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> lcl_pts_fixedmapframe_; // Vector of obstacle points used for sfc generation

  std::unique_ptr<BonxaiT> bonxai_map_; // Bonxai data structure 
  // std::unique_ptr<KD_TREE<pcl::PointXYZ>> kdtree_; // KD-Tree 

  BoolMap3D bool_map_3d_; // Bool map slices 

  std::vector<Eigen::Vector3d> swarm_poses_; // [Comm-less plannig] Pose of other agents 
  std::vector<Eigen::Vector3d> swarm_vels_; // [Comm-less plannig] Velocities of other agents 

  /* Flags */
  bool local_map_updated_{false}; // Indicates if first local map update is done 

  /* Mutexes */
  std::mutex bonxai_map_mtx_;  // Mutex lock for bonxai map
  std::mutex bool_map_3d_mtx_;  // Mutex lock for bool map 3d
  std::mutex lcl_occ_map_mtx_;  // Mutex lock for lcl_pcd_lclmapframe_

  /* Stopwatch for profiling performance */
  // logger_wrapper::Timer tm_update_local_map_{"VoxelMap::updateLocalMap"};  // Time required for map construction
  // logger_wrapper::Timer tm_bonxai_insert_{"bonxai::insertPointCloud"};  // Time required for map construction
  // logger_wrapper::Timer tm_slice_map_{"VoxelMap::getMapSlice"};   // Time required to slice map

  /* Logging */
	std::shared_ptr<logger_wrapper::LoggerWrapper> logger_;

};

/* Setters */
inline void VoxelMap::updateSwarmState(
  const int& id,
  Eigen::Vector3d& swarm_pose, 
  Eigen::Vector3d& swarm_vel)
{
  swarm_poses_[id] = swarm_pose;
  swarm_vels_[id] = swarm_vel;
}

/* Getters */

inline double VoxelMap::getRes() const{ 
  return mp_.resolution_; }

inline Eigen::Vector3d VoxelMap::getGlobalOrigin() const{ 
  return mp_.global_map_origin_; }

inline Eigen::Vector3d VoxelMap::getLocalMapOrigin() const{ 
  return mp_.local_map_origin_; }

inline Eigen::Vector3d VoxelMap::getLocalMapMax() const{ 
  return mp_.local_map_max_; }

inline Eigen::Vector3d VoxelMap::getLocalMapOrigin(const double& offset) const{ 
  return mp_.local_map_origin_ + Eigen::Vector3d::Constant(offset); }

inline Eigen::Vector3d VoxelMap::getLocalMapMax(const double& offset) const{ 
  return mp_.local_map_max_ - Eigen::Vector3d::Constant(offset); }


inline std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> VoxelMap::getLclObsPts()
{
  std::lock_guard<std::mutex> lcl_occ_map_guard(lcl_occ_map_mtx_);
  return lcl_pts_fixedmapframe_;
}


inline double VoxelMap::getInflation() const{ return mp_.static_inflation_; }

inline bool VoxelMap::getBoolMap3D(BoolMap3D& bool_map_3d) {
  if (!local_map_updated_){
    return false;
  }
  
  std::lock_guard<std::mutex> bool_map_3d_guard(bool_map_3d_mtx_);
  bool_map_3d = bool_map_3d_;

  return true;
}

// inline bool VoxelMap::getNearestOccupiedCell(const Eigen::Vector3d &pos, 
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

/* Checks */

inline bool VoxelMap::isTimeout(const double& last_state_time, const double& threshold){
  return (node_->get_clock()->now().seconds() - last_state_time) >= threshold;
} 

inline bool VoxelMap::isInGlobalMap(const Eigen::Vector3d &pos)
{
  if (pos(0) >= -mp_.global_map_size_(0)/2 && pos(0) < mp_.global_map_size_(0)/2
    && pos(1) >= -mp_.global_map_size_(1)/2 && pos(1) < mp_.global_map_size_(1)/2
    && pos(2) >= -mp_.global_map_size_(2)/2 && pos(2) < mp_.global_map_size_(2)/2)
  {
    return true;
  }

  return false;
}

inline bool VoxelMap::isInLocalMap(const Eigen::Vector3d &pos)
{
  if (pos(0) >= mp_.local_map_origin_(0)   && pos(0) < mp_.local_map_max_(0)
    && pos(1) >= mp_.local_map_origin_(1)  && pos(1) < mp_.local_map_max_(1)
    && pos(2) >= mp_.local_map_origin_(2)  && pos(2) < mp_.local_map_max_(2))
  {
    return true;
  }

  return false;
}

inline bool VoxelMap::isOccupied(const Eigen::Vector3d &pos)
{
    // If not in map or not in octree bounding box. return -1 
    if (!isInGlobalMap(pos)){
      return true;
    }

    Bonxai::CoordT coord = bonxai_map_->grid().posToCoord(pos(0), pos(1), pos(2));

    return bonxai_map_->isOccupied(coord);
}

}  // namespace voxel_map

#endif  // VOXEL_MAP__VOXEL_MAP_HPP_
