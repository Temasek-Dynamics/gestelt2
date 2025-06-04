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
        // void odomSubCB(const nav_msgs::msg::Odometry::UniquePtr msg);

    private:
	    rclcpp::CallbackGroup::SharedPtr reentrant_cb_grp_;

        /* Publishers, subscribers, timers and services */
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sensor_pub_; // Publisher of sensor point cloud
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_global_pub_; // Publisher of sensor point cloud

        // rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;               // Subscriber to odometry

        rclcpp::TimerBase::SharedPtr tf_listen_timer_;	    // Timer for planning front end path
        rclcpp::TimerBase::SharedPtr sensor_update_timer_;	    // Timer for planning front end path

        /* TF2 */
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};

        /* Params */
        int num_drones_{1}; // Number of drones, used to create subscription for odom

        std::string global_frame_; // Global map frame
        std::string map_frame_; // Fixed map origin frame
        std::string sensor_frame_;  // Frame of sensor on UAV

        bool voxel_filter_enable_; // True if downsampler is active
        double voxel_size_; // downsampling voxel size

        /* Data */
        pcl::PointCloud<pcl::PointXYZ>::Ptr fake_map_cloud_; // [global frame] Global point cloud map
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_sensor_; //[Sensor frame] Point cloud from fake laser
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_global_; //[Global frame] Point cloud from fake laser
            
        // sensor frame to global frame homogenous transformation matrix
        Eigen::Matrix4d global_to_sensor_mat_{Eigen::Matrix4d::Identity(4, 4)}; 
        Eigen::Matrix4d sensor_to_global_mat_{Eigen::Matrix4d::Identity(4, 4)}; 

        /* Voxel filter for downsampling*/
        std::shared_ptr<pcl::VoxelGrid<pcl::PointXYZ>> vox_grid_{nullptr};
        std::shared_ptr<pcl::PassThrough<pcl::PointXYZ>> pass_fil_x_{nullptr};
        std::shared_ptr<pcl::PassThrough<pcl::PointXYZ>> pass_fil_y_{nullptr};
        std::shared_ptr<pcl::PassThrough<pcl::PointXYZ>> pass_fil_z_{nullptr};

private:
        SensorRenderer sensor_renderer_; // Laser object for rendering fake point clouds
};

#endif // FAKE_SENSOR_HPP_