#ifndef FAKE_SENSOR_HPP_
#define FAKE_SENSOR_HPP_

#include <mutex>

#include <Eigen/Eigen>

#include <pcl_conversions/pcl_conversions.h>
// #include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/point_types.h>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/exceptions.h>

#include <fake_sensor/sensor_renderer.hpp>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace std::placeholders;

class FakeSensor : public rclcpp::Node
{
    public:
        FakeSensor();

        ~FakeSensor();

        // Main timer for updating UAV state 
        void TFListenCB();

        // Main timer for refreshing sensor for rendering point clouds 
        void sensorUpdateTimerCB();

        /* Subscription callbacks */
        void odomSubCB(const nav_msgs::msg::Odometry::UniquePtr msg);

        void swarmOdomCB(const nav_msgs::msg::Odometry::UniquePtr& msg, int id);

    private:
	    rclcpp::CallbackGroup::SharedPtr reentrant_cb_grp_;

        /* Publishers, subscribers, timers and services */
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr sensor_pc_pub_; // Publisher of sensor point cloud

        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;               // Subscriber to odometry
        std::vector<rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr> 
            swarm_odom_subs_;  // Subscription to odometry from other agents

        rclcpp::TimerBase::SharedPtr tf_listen_timer_;	    // Timer for planning front end path
        rclcpp::TimerBase::SharedPtr sensor_update_timer_;	    // Timer for planning front end path

        /* TF2 */
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};

        /* Params */
        int drone_id_{-1};    // ID of drone

        int num_drones_{1}; // Number of drones, used to create subscription for odom

        std::string global_frame_; // Global map frame
        std::string map_frame_; // Fixed map origin frame
        std::string sensor_frame_;  // Frame of sensor on UAV

        bool listen_to_tf_; // If true, then use /tf to determine camera to map transform, else use the odom topic

        bool voxel_filter_enable_; // True if downsampler is active
        double voxel_size_; // downsampling voxel size

        /* Data */
        pcl::PointCloud<pcl::PointXYZ>::Ptr fake_map_cloud_; // [map_frame] Global point cloud map
        pcl::PointCloud<pcl::PointXYZ>::Ptr sensor_cloud_; //[map_frame] Point cloud from fake laser

        std::vector<Eigen::Vector3d> swarm_poses_;

        geometry_msgs::msg::TransformStamped sensor_to_map_tf_; // sensor to global frame transform
            
        Eigen::Matrix4d gbl_to_map_tf_mat_; // global to map TF matrix
        Eigen::Matrix4d map_to_sensor_tf_mat_; // map to sensor TF matrix
        Eigen::Matrix4d sensor_to_map_tf_mat_; // map to sensor TF matrix

        nav_msgs::msg::Odometry cur_odom_;// Last received odom 
        
        /* Flags */
        bool cam_tf_valid_{false}; // True if camera transform is valid
        bool pose_rcv_{false}; // True if camera transform is valid

        /* Mutexes */
        std::mutex odom_mutex_; // Mutex for pose_ data
        std::mutex sensor_tf_mutex_;    // Mutex for sensor_to_map_tf_ transform object

        /* Voxel filter for downsampling*/
        std::shared_ptr<pcl::VoxelGrid<pcl::PointXYZ>> vox_grid_{nullptr};
        std::shared_ptr<pcl::PassThrough<pcl::PointXYZ>> pass_fil_x_{nullptr};
        std::shared_ptr<pcl::PassThrough<pcl::PointXYZ>> pass_fil_y_{nullptr};
        std::shared_ptr<pcl::PassThrough<pcl::PointXYZ>> pass_fil_z_{nullptr};

private:
        SensorRenderer sensor_renderer_; // Laser object for rendering fake point clouds
};

#endif // FAKE_SENSOR_HPP_