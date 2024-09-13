#ifndef VOXEL_MAP__VOXEL_MAP_HPP_
#define VOXEL_MAP__VOXEL_MAP_HPP_

#include "voxel_map/visibility_control.h"

#include <Eigen/Eigen>

#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
// #include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>

#include <nav_msgs/msg/Odometry.h>
// #include <visualization_msgs/msg/Marker.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_broadcaster.h>

#include <bonxai/bonxai.hpp>
#include <bonxai/pcl_utils.hpp>
#include <bonxai/probabilistic_map.hpp>


#include <logger_wrapper/logger_wrapper.hpp>


struct MappingParameters
{
  /* map properties */
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
  int pose_type_;       // Type of pose input (pose, odom or TF)

  double pose_timeout_; // Timeout for pose update before emergency stop is activated

  double max_range;

  /* Cloud downsampler parameters */
  // bool downsample_cloud_; // True if downsampling cloud before input to octree occupancy
  // int depth_stride_; // Number of depth pixels to skip
  // double voxel_size_; // Size of voxel for voxel grid filter

  /* visualization and computation time display */
  double ground_height_; // Lowest possible height (z-axis)

  std::string cam_frame_;
  std::string global_frame_; // frame id of global reference 
  std::string uav_origin_frame_; // frame id of UAV origin

};

struct BoolMap3D {
  int z_separation_cm;   // [cm] Separation between slice layers
  int min_height_cm;     // [cm] Lowest slice height
  int max_height_cm;     // [cm] Highest slice height

  double z_separation_m; // [m] separation between slice layers
  double min_height_m;   // [m] Lowest slice height
  double max_height_m;   // [m] Highest slice height

  Eigen::Vector3d origin{0.0, 0.0, 0.0}; // [m] Origin of local map

  int width;            // [] Number of width cells 
  int height;           // [] Number of height cells
  double resolution;    // [cm] Resolution of map

  std::map<int, std::vector<bool>> bool_maps; // Map of BoolMap objects
};

// intermediate mapping data for fusion

struct MappingData
{
  Eigen::Vector3d cam2body_rpy_deg{0.0, 0.0, 0.0};

  // Homogenous Transformation matrix of camera to body frame
  Eigen::Matrix4d cam2body_{Eigen::Matrix4d::Identity(4, 4)};
  // Homogenous Transformation matrix of body to UAV origin frame
  // NOTE: USE `body2origin_.block<3,1>(0,3)` FOR UAV POSE!
  Eigen::Matrix4d body2origin_{Eigen::Matrix4d::Identity(4, 4)};
  // Homogenous Transformation matrix of camera to UAV origin frame
  Eigen::Matrix4d cam2origin_{Eigen::Matrix4d::Identity(4, 4)};

  // True if pose has been received
  bool has_pose_{false};

  // TODO: Use this to flag timeout
  // True if depth and odom has timed out
  double last_sensor_msg_time{-1.0};

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

namespace voxel_map
{

class VoxelMap
{
public: 

// Custom type definition for message filters
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, nav_msgs::Odometry>
    SyncPolicyCloudOdom;

typedef std::shared_ptr<message_filters::Synchronizer<SyncPolicyCloudOdom>> SynchronizerCloudOdom;

typedef std::shared_ptr<VoxelMap> Ptr;
using BonxaiT = Bonxai::ProbabilisticMap;

public:
  VoxelMap();

  virtual ~VoxelMap();


  // Reset map data
  void reset(const double& resolution);


  // Initialize gridmap for ros
  void initMapROS(ros::NodeHandle &nh, ros::NodeHandle &pnh);

  void initROSPubSubTimers(ros::NodeHandle &nh, ros::NodeHandle &pnh);

  void readROSParams(ros::NodeHandle &nh, ros::NodeHandle &pnh);

  /* Gridmap conversion methods */

  // Get camera-to-global frame transformation
  void getCamToGlobalPose(const geometry_msgs::Pose &pose);
  
  // Convert point cloud message to point cloud map, transform it from camera-to-global frame and save it. 
  void pcdMsgToMap(const sensor_msgs::PointCloud2 &msg);
  
  // Convert point cloud to point cloud map, transform it from camera-to-global frame and save it. 
  void pcdToMap(pcl::PointCloud<pcl::PointXYZ>::Ptr pcd);

  /** Getter methods */

  // Get occupancy grid resolution
  double getRes() { return mp_.resolution_; }

  // Get global map origin (This is defined to be a corner of the global map i.e. (-W/2, -L/2, 0))
  Eigen::Vector3d getGlobalOrigin() { return mp_.global_map_origin_; }

  // Get local map origin (This is defined to be a corner of the local map i.e. (-local_W/2, -local_L/2, 0))
  Eigen::Vector3d getLocalOrigin() { return mp_.local_map_origin_; }

  // Get number of voxels in global map
  Eigen::Vector3i getGlobalMapNumVoxels() const { return mp_.global_map_num_voxels_; }

  // Get number of voxels in local map
  Eigen::Vector3i getLocalMapNumVoxels() const { return mp_.local_map_num_voxels_; }

  // Get inflation value
  double getInflation() const{ return mp_.inflation_; }

  bool getBoolMap3D(BoolMap3D& bool_map_3d) {
    if (!init_bool_map_3d_){
      return false;
    }

    std::lock_guard<std::mutex> bool_map_3d_guard(bool_map_3d_mtx_);

    bool_map_3d = bool_map_3d_;

    return true;
  }

  /* Checks */

  // Checks if time elapsed has exceeded a given threshold
  bool isTimeout(const double& last_state_time, const double& threshold){
    return (ros::Time::now().toSec() - last_state_time) >= threshold;
  } 

  // Checks if camera pose is valid
  bool isPoseValid();

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

private:

  MappingParameters mp_;
  MappingData md_;

  /**
   * Subscriber Callbacks
  */

  // Subscriber callback to point cloud and odom
  void cloudOdomCB( const sensor_msgs::PointCloud2ConstPtr &msg_pc, 
                    const nav_msgs::OdometryConstPtr &msg_odom);

  /**
   * Timer Callbacks
  */

  /**
   * @brief This timer publishes a visualization of the occupancy grid
  */
  void visTimerCB(const ros::TimerEvent & /*event*/);

  /**
   * @brief This timer updates the local map for use by planners
  */
  void updateLocalMapTimerCB(const ros::TimerEvent & /*event*/);

  /**
   * @brief Timer for checking collision of drone with obstacles
   * 
   */
  void checkCollisionsTimerCB(const ros::TimerEvent & /*event*/);

// Frequently used methods
public:

  // Convert from meters to centimeters
  int mToCm(const double& val_m){
    return (int) (val_m * 100.0);
  }

  // Convert from centimeters to meters
  double cmToM(const int& val_cm) {
    return ((double) val_cm)/100.0;  
  }

  /**
   * @brief Get all point clouds at a slice of the local map centered at height z with specified thickness
   * 
   * @param slice_z [cm] Slice at height z
   * @param thickness [m] Thickness of the map
   * @return gestelt_msgs::BoolMap 
   */
  std::vector<bool> sliceMap(const double& slice_z_cm, const double& thickness);

  /* Gridmap operation methods */

  // Called by planners to update the local map
  void updateLocalMap();

  // True if given GLOBAL position is within the GLOBAL map boundaries, else False
  bool isInGlobalMap(const Eigen::Vector3d &pos)
  {
    if (pos(0) >= -mp_.global_map_size_(0)/2 && pos(0) < mp_.global_map_size_(0)/2
      && pos(1) >= -mp_.global_map_size_(1)/2 && pos(1) < mp_.global_map_size_(1)/2
      && pos(2) >= -mp_.global_map_size_(2)/2 && pos(2) < mp_.global_map_size_(2)/2)
    {
      return true;
    }

    return false;
  }

  // True if given GLOBAL position is within the LOCAL map boundaries, else False
  bool isInLocalMap(const Eigen::Vector3d &pos)
  {
    if (pos(0) >= mp_.local_map_origin_(0)   && pos(0) < mp_.local_map_max_(0)
      && pos(1) >= mp_.local_map_origin_(1)  && pos(1) < mp_.local_map_max_(1)
      && pos(2) >= mp_.local_map_origin_(2)  && pos(2) < mp_.local_map_max_(2))
    {
      return true;
    }

    return false;
  }

  // Get occupancy value of given position in Occupancy grid
  bool getOccupancy(const Eigen::Vector3d &pos){
    // If not in map or not in octree bounding box. return -1 
    if (!isInGlobalMap(pos)){
      return true;
    }

    Bonxai::CoordT coord = bonxai_map_->grid().posToCoord(pos(0), pos(1), pos(2));

    return bonxai_map_->isOccupied(coord);
  }


  // Check if current index is free
  bool isFree(const Eigen::Vector3i& idx) {
    return !isOccupied(idx);
  }

  // Check if current index is occupied
  bool isOccupied(const Eigen::Vector3i& idx) {
    std::cout << "dynvoro->isOccupied(" << idx.transpose() << ")" << std::endl;
    if (!isInGlobalVoxelMap(idx)){
      return true; 
    }

    Eigen::Vector3d pos = intToFloat(idx);
    Bonxai::CoordT coord = bonxai_map_->grid().posToCoord(pos(0), pos(1), pos(2));

    return bonxai_map_->isOccupied(coord);
  }

  // Check if index is within map bounds
  bool isInLocalVoxelMap(const Eigen::Vector3i& idx) {
    if (idx(0) >= 0 && idx(0) < mp_.local_map_num_voxels_(0)
      && idx(1) >= 0 && idx(1) < mp_.local_map_num_voxels_(1)
      && idx(2) >= 0 && idx(2) < mp_.local_map_num_voxels_(2))
    {
      return true;
    }
    return false;
  }

  // Check if index is within map bounds
  bool isInGlobalVoxelMap(const Eigen::Vector3i& idx) {
    if (idx(0) >= 0 && idx(0) < mp_.global_map_num_voxels_(0)
      && idx(1) >= 0 && idx(1) < mp_.global_map_num_voxels_(1)
      && idx(2) >= 0 && idx(2) < mp_.global_map_num_voxels_(2))
    {
      return true;
    }
    return false;
  }

  /// Float position to discrete cell coordinate
  Eigen::Vector3i floatToInt(const Eigen::Vector3d &pos) {

    // Eigen::Vector3i idx = (((pos - getGlobalOrigin()) / getRes() ) - Eigen::Vector3d::Constant(0.5)).cast<int>() ;
    Eigen::Vector3i idx = ((pos - getGlobalOrigin()) / getRes() ).cast<int>() ;
    return idx;
  }

  /// Discrete cell coordinate to float position
  Eigen::Vector3d intToFloat(const Eigen::Vector3i &idx) {
    // return (idx.template cast<double>() + Vecf<3>::Constant(0.5)) * getRes() + getGlobalOrigin();

    // Eigen::Vector3d pos = (idx.cast<double>() + Eigen::Vector3d::Constant(0.5))  * getRes() + getGlobalOrigin();
    Eigen::Vector3d pos = (idx.cast<double>())  * getRes() + getGlobalOrigin();
    return pos;
  }

private: 
  /* Params */
  bool verbose_print_{false}; // Flag to enable printing of debug information such as timers
  bool dbg_input_entire_map_{false}; // flag to indicate that map will be constructed at the start from the entire pcd map (instead of through incremental sensor data)
  std::string entire_pcd_map_topic_; // Topic to listen for an entire PCD for debugging

  bool check_collisions_{true}; // Flag for checking collisions
  
  double col_warn_radius_, col_fatal_radius_; // collision check radius

  double viz_occ_map_freq_{-1.0}; // Frequency to publish occupancy map visualization
  double update_local_map_freq_{-1.0};  // Frequency to update local map

  /* ROS Publishers, subscribers and Timers */

  // Message filters for point cloud/depth camera and pose/odom
  SynchronizerCloudOdom sync_cloud_odom_;
  
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>> cloud_sub_;
  std::shared_ptr<message_filters::Subscriber<nav_msgs::msg::Odometry>> odom_sub_;

  // Message filters for point cloud and tf
  std::shared_ptr<tf2_ros::MessageFilter<sensor_msgs::PointCloud2>> tf_cloud_filter_;

  // ros::Publisher occ_map_pub_; // Publisher for occupancy map
	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr occ_map_pub_;
  
  // ros::Publisher slice_map_pub_; // Publisher for z slice of map
  // ros::Publisher collision_viz_pub_; // Publisher for collision visualization spheres
  // ros::Publisher local_map_poly_pub_; // Publisher to show local map bounds

	rclcpp::TimerBase::SharedPtr vis_occ_timer_;	 // Timer for visualization
	rclcpp::TimerBase::SharedPtr check_collisions_timer_; // Timer for checking collisions
	rclcpp::TimerBase::SharedPtr update_local_map_timer_; // Timer for updating local map

  // TF transformation 
  tf2_ros::Buffer tfBuffer_; 
  tf2_ros::TransformBroadcaster tf_broadcaster_; // broadcast tf link

  /* Data structures for point clouds */
  pcl::PointCloud<pcl::PointXYZ>::Ptr local_occ_map_pts_; // (In local frame) Occupancy map points formed by Bonxai probabilistic mapping (w.r.t local map origin)
  pcl::PointCloud<pcl::PointXYZ>::Ptr local_global_occ_map_pts_; // (In global frame) Occupancy map points formed by Bonxai probabilistic mapping (w.r.t local map origin)
  pcl::PointCloud<pcl::PointXYZ>::Ptr global_map_in_origin_;  // Point cloud global map in UAV Origin frame

  BoolMap3D bool_map_3d_; // Bool map slices 

  // nav_msgs::Odometry odom_msg_; // Odom message
  std::unique_ptr<BonxaiT> bonxai_map_; // Bonxai data structure 

  /* Logic flags*/
  bool init_bool_map_3d_{false};

  /* Mutexes */
  std::mutex bool_map_3d_mtx_;  // Mutex lock for bool map 3d

  /* Stopwatch for profiling performance */
  Timer tm_bonxai_insert_{"bonxai->insertPointCloud"};
  Timer tm_slice_map_{"grid_map.sliceMap"};

};

}  // namespace voxel_map

#endif  // VOXEL_MAP__VOXEL_MAP_HPP_
