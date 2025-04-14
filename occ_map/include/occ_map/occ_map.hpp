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
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/robot_utils.hpp"

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
    return resolution_;
  }

  // Get global map origin (This is defined to be a corner of the global map i.e. (-W/2, -L/2, 0))
  inline Eigen::Vector3d getGlobalMapSize() const{
    return global_map_size_; 
  }

  // Get local map origin (This is defined to be a corner of the local map i.e. (-local_W/2, -local_L/2, 0))
  inline Eigen::Vector3d getLocalMapOrigin() const{
    return local_map_origin_;
  }

  inline Eigen::Vector3d getLocalMapMax() const{
    return local_map_max_;
  }

  inline Eigen::Vector3d getLocalMapOrigin(const double& offset = 0.0) const{
    return local_map_origin_ + Eigen::Vector3d::Constant(offset);
  }

  inline Eigen::Vector3d getLocalMapMax(const double& offset = 0.0) const{
    return local_map_max_ - Eigen::Vector3d::Constant(offset); 
  }

  inline double getTransformTolerance() const {
    return transform_tolerance_;
  }

  inline double getLocalMapMaxExtent() const {
    return local_map_size_(0);
  }

  /**
   * @brief  Get the occupancy map's use_radius_ parameter, corresponding to
   * whether the footprint for the robot is a circle with radius robot_radius_
   * or an arbitrarily defined footprint in footprint_.
   * @return  use_radius_
   */
  bool getUseRadius() {return use_radius_;}

  std::shared_ptr<tf2_ros::Buffer> getTfBuffer() {return tf_buffer_;}

  bool getRobotPose(geometry_msgs::msg::PoseStamped & global_pose)
  {
    return nav2_util::getCurrentPose(
      global_pose, *tf_buffer_,
      global_frame_, base_link_frame_, transform_tolerance_);
  }

  bool transformPoseToGlobalFrame(
    const geometry_msgs::msg::PoseStamped & input_pose,
    geometry_msgs::msg::PoseStamped & transformed_pose)
  {
    if (input_pose.header.frame_id == global_frame_) {
      transformed_pose = input_pose;
      return true;
    } 
    else {
      return nav2_util::transformPoseInTargetFrame(
        input_pose, transformed_pose, *tf_buffer_,
        global_frame_, transform_tolerance_);
    }
  }

  bool transformPoseToTargetFrame(
    const std::string& target_frame,
    const geometry_msgs::msg::PoseStamped & input_pose,
    geometry_msgs::msg::PoseStamped & transformed_pose)
  {

    if (input_pose.header.frame_id == target_frame) {
      transformed_pose = input_pose;
      return true;
    }
    else {
      return nav2_util::transformPoseInTargetFrame(
        input_pose, transformed_pose, *tf_buffer_,
        target_frame, transform_tolerance_);
    }
  }

  /**
   * @brief Get local points in map frame (in fixed map frame).
   * Used by safe flight corridor generation
   * 
   * @return std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> 
   */
  inline std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> getLocalPtsInMapFrame(){
    std::lock_guard<std::mutex> lcl_occ_map_guard(lcl_occ_map_mtx_);
    return lcl_pts_map_;
  }

  // Takes in position [global_frame] and check if within global map
  inline bool inGlobalMap(const Eigen::Vector3d &pos){
    return (pos(0) >= -global_map_size_(0)/2 
      && pos(0) < global_map_size_(0)/2
      && pos(1) >= -global_map_size_(1)/2 
      && pos(1) < global_map_size_(1)/2
      && pos(2) >= 0.0 
      && pos(2) < global_map_size_(2));
  }

  // Takes in position [global_frame] and check if within global map
  inline bool inGlobalMapIdx(const Eigen::Vector3i &idx){
    return (idx(0) >= -global_map_size_idx_(0)/2 
      && idx(0) < global_map_size_idx_(0)/2
      && idx(1) >= -global_map_size_idx_(1)/2 
      && idx(1) < global_map_size_idx_(1)/2
      && idx(2) >= 0 
      && idx(2) < global_map_size_idx_(2));
  }

  inline bool inLocalMap(const Eigen::Vector3d &pos){
    return (pos(0) >= local_map_origin_(0)   
      && pos(0) < local_map_max_(0)
      && pos(1) >= local_map_origin_(1)  
      && pos(1) < local_map_max_(1)
      && pos(2) >= local_map_origin_(2)  
      && pos(2) < local_map_max_(2));
  }

  inline Eigen::Vector3i posToIdx(const Eigen::Vector3d &pos){
    return Eigen::Vector3i{ 
      static_cast<int32_t>(pos(0) * inv_resolution_ - std::signbit(pos(0))),
      static_cast<int32_t>(pos(1) * inv_resolution_ - std::signbit(pos(1))),
      static_cast<int32_t>(pos(2) * inv_resolution_ - std::signbit(pos(2)))};
  }

  inline Eigen::Vector3d idxToPos(const Eigen::Vector3i &idx){
    return Eigen::Vector3d{(static_cast<double>(idx(0)) + 0.5) * resolution_,
                            (static_cast<double>(idx(1)) + 0.5) * resolution_,
                            (static_cast<double>(idx(2)) + 0.5) * resolution_};
  }

  bool mapToWorld(const Eigen::Vector3d& pos_map, Eigen::Vector3d& pos_world){
    pos_world = (map_to_world_mat_ * pos_map.homogeneous()).hnormalized(); 

    return true;
  }

  bool worldToMap(const Eigen::Vector3d& pos_world, Eigen::Vector3d& pos_map){
    if (!inGlobalMap(pos_world)){
      return false;
    }

    pos_map = (world_to_map_mat_ * pos_world.homogeneous()).hnormalized();  

    return true;
  }

  /**
   * @brief Gets the cost of a given position in map coordinates
   * 
   * @param pos 
   * @return int 
   */
  int getCost(const Eigen::Vector3d &pos);

    /**
   * @brief Gets the cost of a given position in pixel coordinates
   * 
   * @param pos 
   * @return int 
   */
  int getCostIdx(const Eigen::Vector3i &idx);

  std::string getMapFrameID() const{
    return map_frame_;
  }

  std::string getGlobalFrameID() const{
    return global_frame_;
  }

  // Provide a typedef to ease future code maintenance
  typedef std::recursive_mutex mutex_t;
  mutex_t * getMutex()
  {
    return access_;
  }
  
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

  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor1_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor2_;
  std::unique_ptr<nav2_util::NodeThread> executor_thread1_;
  std::unique_ptr<nav2_util::NodeThread> executor_thread2_;

	/* Callback groups */
	rclcpp::CallbackGroup::SharedPtr mtex_callback_grp1_;
	rclcpp::CallbackGroup::SharedPtr mtex_callback_grp2_;

  std::string name_;
  std::string parent_namespace_;

  /* Params */
  int drone_id_{0}; //Drone ID

  bool print_timer_{false}; // Flag to enable printing of debug information such as timers

  double initial_transform_timeout_{0.0};
  double transform_tolerance_{0.5};

  bool use_radius_{true};

  // Noise filter
  double noise_search_radius_{0.3}; // Radius to search for neighbouring points
  int noise_min_neighbors_{1}; // Minimum number of neighbors within 'search_radius' to be a valid point

  // Passthrough filter for point cloud 
  double cloud_in_min_z_{0.0};
  double cloud_in_max_z_{10.0};

  double time_vel_{0.1}; // [s] time along velocity vector to mark as occupied

  double viz_occ_map_freq_{-1.0}; // Frequency to publish occupancy map visualization
  double update_local_map_freq_{-1.0};  // Frequency to update local map

  std::string global_frame_; // frame id of global/inertial reference 
  std::string map_frame_; // frame id of fixed map frame
  std::string camera_frame_; // frame id of camera link
  std::string base_link_frame_; // frame id of base_link

  // Local and global map are bounded 3d boxes
  Eigen::Vector3d local_map_origin_{0.0, 0.0, 0.0}; // Origin of local map (Set to be the corner of the map) w.r.t map frame
  Eigen::Vector3d local_map_max_{0.0, 0.0, 0.0}; // max position of local map (Set to be the corner of the map)

  Eigen::Vector3i global_map_size_idx_{0, 0, 0}; //  Size of global occupancy map in voxels (voxels)
  Eigen::Vector3d global_map_size_{0.0, 0.0, 0.0}; //  Size of global occupancy map  (m)
  Eigen::Vector3d local_map_size_{0.0, 0.0, 0.0}; //  Size of local occupancy map (m)

  Eigen::Matrix4d world_to_map_mat_{Eigen::Matrix4d::Identity(4, 4)};
  Eigen::Matrix4d map_to_world_mat_{Eigen::Matrix4d::Identity(4, 4)};

  double resolution_{0.0};   // Also defined as the size of each individual voxel                 
  double inv_resolution_{0.0};   // Also defined as the size of each individual voxel                 
  double inflation_{0.0};    // Static obstacle inflation in map units
  int inflation_voxels_{0};  // Inflation in number of voxel 
  double max_range_{0.0}; // Max sensor range

  // [DYNAMIC]: Homogenous Transformation matrix of camera to fixed map frame
  Eigen::Matrix4d cam_to_map_mat_{Eigen::Matrix4d::Identity(4, 4)};

  // [DYNAMIC]: Homogenous Transformation matrix of fbase_link to map frame
  Eigen::Matrix4d bl_to_map_mat_{Eigen::Matrix4d::Identity(4, 4)};

  double last_cloud_cb_time_{-1.0}; // True if cloud and odom has timed out

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
  // [GLOBAL FRAME] Locally bounded point clouds from Bonxai probabilistic mapping 
  std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> lcl_pcd_map_raw_; 
  // [GLOBAL FRAME] lcl_pcd_map_raw_ after post-processing to remove noise
  std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> lcl_pcd_map_; 

  // Vector of obstacle points used for sfc generation
  std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> lcl_pts_map_; // Local obstacle points in map frame

  std::shared_ptr<Bonxai::ProbabilisticMap> bonxai_map_; // Bonxai data structure 
  std::unique_ptr<KD_TREE<pcl::PointXYZ>> kdtree_; // KD-Tree 

  /* Flags */
  bool local_map_updated_{false}; // Indicates if first local map update is done 

  /* Mutexes */
  mutex_t * access_;

  std::mutex bonxai_map_mtx_;  // Mutex lock for bonxai map
  std::mutex lcl_occ_map_mtx_;  // Mutex lock for 

  /* Stopwatch for profiling performance */
  // logger_wrapper::Timer tm_update_local_map_{"OccMap::updateLocalMap"};  // Time required for map construction
  // logger_wrapper::Timer tm_bonxai_insert_{"bonxai::insertPointCloud"};  // Time required for map construction

  /* Point cloud Filters */
  pcl::PassThrough<pcl::PointXYZ> z_filter_cloud_in_;

  /* Logging */
	std::shared_ptr<logger_wrapper::LoggerWrapper> logger_;
};

}  // namespace occ_map

#endif  // OCC_MAP__OCC_MAP_HPP_
