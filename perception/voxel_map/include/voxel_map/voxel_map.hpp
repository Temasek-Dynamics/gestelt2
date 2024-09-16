#ifndef VOXEL_MAP__VOXEL_MAP_HPP_
#define VOXEL_MAP__VOXEL_MAP_HPP_

#include "voxel_map/visibility_control.h"

#include <Eigen/Eigen>

#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/wait_for_message.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>

/* Debugging */
#include <visualization_msgs/msg/marker.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <tf2_ros/transform_broadcaster.h>

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
    Eigen::Vector3d local_map_origin_; // Origin of local map (Set to be the corner of the map)
    Eigen::Vector3d local_map_max_; // max position of local map (Set to be the corner of the map)
    
    Eigen::Vector3d global_map_size_; //  Size of global occupancy map  (m)
    Eigen::Vector3d local_map_size_; //  Size of local occupancy map (m)

    Eigen::Vector3i global_map_num_voxels_; //  Size of global occupancy grid (no. of voxels)
    Eigen::Vector3i local_map_num_voxels_; //  Size of local occupancy grid (no. of voxels)

    double resolution_;   // Also defined as the size of each individual voxel                 
    double inflation_;    // Inflation in units of meters
    int inf_num_voxels_;  // Inflation in units of number of voxels, = inflation_/resolution_ 

    double pose_timeout_; // Timeout for pose update before emergency stop is activated

    double max_range; // Max sensor range

    /* visualization and computation time display */
    double ground_height_; // Lowest possible height (z-axis)

    std::string sensor_frame_; // Frame id of sensor
    std::string global_frame_; // frame id of global reference 
    std::string uav_origin_frame_; // frame id of UAV origin
  };

  /* Dynamic data used during mapping */
  struct MappingData
  {
    bool has_pose_{false}; // Indicates if pose has been received

    Eigen::Vector3d cam2body_rpy_deg{0.0, 0.0, 0.0};

    // Homogenous Transformation matrix of camera to body frame
    Eigen::Matrix4d cam2body_{Eigen::Matrix4d::Identity(4, 4)};
    // Homogenous Transformation matrix of body to UAV origin frame
    // NOTE: USE `body2origin_.block<3,1>(0,3)` FOR UAV POSE!
    Eigen::Matrix4d body2origin_{Eigen::Matrix4d::Identity(4, 4)};
    // Homogenous Transformation matrix of camera to UAV origin frame
    Eigen::Matrix4d cam2origin_{Eigen::Matrix4d::Identity(4, 4)};

    double last_sensor_msg_time{-1.0}; // True if cloud and odom has timed out

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

  /* Boolean map structure used by planner*/
  struct BoolMap3D {  
    int z_separation_cm{-1};   // [cm] Separation between slice layers
    int min_height_cm{-1};     // [cm] Lowest slice height
    int max_height_cm{-1};     // [cm] Highest slice height

    double z_separation_m{-1.0}; // [m] separation between slice layers
    double min_height_m{-1.0};   // [m] Lowest slice height
    double max_height_m{-1.0};   // [m] Highest slice height

    Eigen::Vector3d origin{0.0, 0.0, 0.0}; // [m] Origin of local map

    int width{-1};            // [] Number of width cells 
    int height{-1};           // [] Number of height cells
    double resolution{-1.0};    // [cm] Resolution of map

    std::map<int, std::vector<bool>> bool_maps; // Map of BoolMap objects
  };

class VoxelMap : public rclcpp::Node
{
public: 
  // Custom type definition for message filters
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, nav_msgs::msg::Odometry>
      SyncPolicyCloudOdom;
  typedef std::shared_ptr<message_filters::Synchronizer<SyncPolicyCloudOdom>> SynchronizerCloudOdom;

  // Custom type definition for Bonxai
  typedef std::shared_ptr<VoxelMap> Ptr;
  using BonxaiT = Bonxai::ProbabilisticMap;

public:

  /** Initialization methods */

  VoxelMap();

  virtual ~VoxelMap();

  // Reset map data
  void reset(const double& resolution);

  void init();

  void initParams();

  void initPubSubTimer();

  /* Core methods */

  // Get camera-to-global frame transformation
  void getCamToGlobalPose(const geometry_msgs::msg::Pose &pose);
  
  // Convert point cloud message to point cloud map, transform it from camera-to-global frame and save it. 
  void pcd2MsgToMap(const sensor_msgs::msg::PointCloud2& msg);
  
  // Convert point cloud to point cloud map, transform it from camera-to-global frame and save it. 
  void pcdToMap(const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> pcd);

  // Called by planners to update the local map
  void updateLocalMap();

  /**
   * @brief Get all point clouds at a slice of the local map centered at height z with specified thickness
   * 
   * @param slice_z [cm] Slice at height z
   * @param thickness [m] Thickness of the map
   * @return boolean map array
   */
  std::vector<bool> sliceMap(const double& slice_z_cm, const double& thickness);

  /** Publisher methods */

  /**
   * @brief Publish map for visualization
   * 
   */
  void publishOccMap(const pcl::PointCloud<pcl::PointXYZ>::Ptr& occ_map_pts);

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

  // Subscriber callback to point cloud and odom
  void cloudOdomCB( const sensor_msgs::msg::PointCloud2::SharedPtr msg_pc, 
                    const nav_msgs::msg::Odometry::SharedPtr msg_odom);

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
   * @brief Timer for checking collision of drone with obstacles
   * 
   */
  void checkCollisionsTimerCB();

private:

  /* Getter methods */

  // Get occupancy grid resolution
  double getRes() const;

  // Get global map origin (This is defined to be a corner of the global map i.e. (-W/2, -L/2, 0))
  Eigen::Vector3d getGlobalOrigin() const;

  // Get local map origin (This is defined to be a corner of the local map i.e. (-local_W/2, -local_L/2, 0))
  Eigen::Vector3d getLocalOrigin() const;

  // Get number of voxels in global map
  Eigen::Vector3i getGlobalMapNumVoxels() const;

  // Get number of voxels in local map
  Eigen::Vector3i getLocalMapNumVoxels() const;

  // Get inflation value
  double getInflation() const;

  bool getBoolMap3D(BoolMap3D& bool_map_3d);

  /* Checks */

  // Checks if time elapsed has exceeded a given threshold
  bool isTimeout(const double& last_state_time, const double& threshold);

  // Checks if camera pose is valid
  bool isPoseValid();

  // True if given GLOBAL position is within the GLOBAL map boundaries, else False
  bool isInGlobalMap(const Eigen::Vector3d &pos);

  // True if given GLOBAL position is within the LOCAL map boundaries, else False
  bool isInLocalMap(const Eigen::Vector3d &pos);

  /* Helper methods */

  // Convert from meters to centimeters
  int mToCm(const double& val_m);

  // Convert from centimeters to meters
  double cmToM(const int& val_cm);

private: 
  /* Params */
  int drone_id_{0}; //Drone ID

  bool verbose_print_{false}; // Flag to enable printing of debug information such as timers
  bool dbg_input_entire_map_{false}; // flag to indicate that map will be constructed at the start from the entire pcd map (instead of through incremental sensor data)
  std::string entire_pcd_map_topic_; // Topic to listen for an entire PCD for debugging

  bool check_collisions_{true}; // Flag for checking collisions
  double col_warn_radius_, col_fatal_radius_; // collision check radius

  double viz_occ_map_freq_{-1.0}; // Frequency to publish occupancy map visualization
  double update_local_map_freq_{-1.0};  // Frequency to update local map
  double check_col_freq_{-1.0};  // Frequency to update local map

  MappingParameters mp_;  // Parameters used for map

  /* Subscribers */

  SynchronizerCloudOdom sync_cloud_odom_; // Synchronization policy for cloud and odom topic
  
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>> cloud_sub_;
  std::shared_ptr<message_filters::Subscriber<nav_msgs::msg::Odometry>> odom_sub_;

  /* Publishers  */
	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr occ_map_pub_;
  
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr slice_map_pub_; // Publisher for z slice of map
  rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr local_map_bounds_pub_; // Publisher to show local map bounds
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr collision_viz_pub_; // Publisher for collision visualization spheres

  /* Timers */
	rclcpp::TimerBase::SharedPtr viz_map_timer_;	 // Timer for visualizing map
	rclcpp::TimerBase::SharedPtr check_collisions_timer_; // Timer for checking collisions
	rclcpp::TimerBase::SharedPtr update_local_map_timer_; // Timer for updating local map

  // TF transformation 
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_; // broadcast tf link from global map frame to local map origin 
 
  /* Data structures for maps */
  MappingData md_;  // Mapping data

  std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> local_occ_map_pts_; // (In local frame) Occupancy map points formed by Bonxai probabilistic mapping (w.r.t local map origin)
  std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> local_global_occ_map_pts_; // (In global frame) Occupancy map points formed by Bonxai probabilistic mapping (w.r.t local map origin)
  std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> global_map_in_origin_;  // Point cloud global map in UAV Origin frame

  std::unique_ptr<BonxaiT> bonxai_map_; // Bonxai data structure 

  BoolMap3D bool_map_3d_; // Bool map slices 

  /* Mutexes */
  std::mutex bool_map_3d_mtx_;  // Mutex lock for bool map 3d
  std::mutex lcl_occ_map_mtx_;  // Mutex lock for local_occ_map_pts_

  /* Stopwatch for profiling performance */
  logger_wrapper::Timer tm_update_local_map_{"VoxelMap::updateLocalMap"};  // Time required for map construction
  logger_wrapper::Timer tm_bonxai_insert_{"bonxai::insertPointCloud"};  // Time required for map construction
  logger_wrapper::Timer tm_slice_map_{"VoxelMap::sliceMap"};   // Time required to slice map

  /* Logging */
	std::shared_ptr<logger_wrapper::LoggerWrapper> logger_;

};

/* Getters */

double VoxelMap::getRes() const{ return mp_.resolution_; }

Eigen::Vector3d VoxelMap::getGlobalOrigin() const{ return mp_.global_map_origin_; }

Eigen::Vector3d VoxelMap::getLocalOrigin() const{ return mp_.local_map_origin_; }

Eigen::Vector3i VoxelMap::getGlobalMapNumVoxels() const { return mp_.global_map_num_voxels_; }

Eigen::Vector3i VoxelMap::getLocalMapNumVoxels() const { return mp_.local_map_num_voxels_; }

double VoxelMap::getInflation() const{ return mp_.inflation_; }

bool VoxelMap::getBoolMap3D(BoolMap3D& bool_map_3d) {
  if (bool_map_3d_.z_separation_cm < 0){
    return false;
  }
  {
    std::lock_guard<std::mutex> bool_map_3d_guard(bool_map_3d_mtx_);
    bool_map_3d = bool_map_3d_;
  }

  return true;
}

/* Checks */

bool VoxelMap::isTimeout(const double& last_state_time, const double& threshold){
  return (this->get_clock()->now().seconds() - last_state_time) >= threshold;
} 

bool VoxelMap::isInGlobalMap(const Eigen::Vector3d &pos)
{
  if (pos(0) >= -mp_.global_map_size_(0)/2 && pos(0) < mp_.global_map_size_(0)/2
    && pos(1) >= -mp_.global_map_size_(1)/2 && pos(1) < mp_.global_map_size_(1)/2
    && pos(2) >= -mp_.global_map_size_(2)/2 && pos(2) < mp_.global_map_size_(2)/2)
  {
    return true;
  }

  return false;
}

bool VoxelMap::isInLocalMap(const Eigen::Vector3d &pos)
{
  if (pos(0) >= mp_.local_map_origin_(0)   && pos(0) < mp_.local_map_max_(0)
    && pos(1) >= mp_.local_map_origin_(1)  && pos(1) < mp_.local_map_max_(1)
    && pos(2) >= mp_.local_map_origin_(2)  && pos(2) < mp_.local_map_max_(2))
  {
    return true;
  }

  return false;
}

/* Helper */

int VoxelMap::mToCm(const double& val_m){
  return (int) (val_m * 100.0);
}

double VoxelMap::cmToM(const int& val_cm) {
  return ((double) val_cm)/100.0;  
}


}  // namespace voxel_map

#endif  // VOXEL_MAP__VOXEL_MAP_HPP_
