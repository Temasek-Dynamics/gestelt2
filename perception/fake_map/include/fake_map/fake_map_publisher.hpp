#ifndef FAKE_MAP_PUBLISHER_HPP_
#define FAKE_MAP_PUBLISHER_HPP_
/**
 * @file fake_map_publisher.hpp
 * @author John Tan (john_tanguanzhong@hotmail.com)
 * @brief Publishes a complete map in the form of sensor_msgs/PointCloud2, 
 * primarily used for debugging planners
 * @version 0.1
 * @date 2024-09-20
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include <rclcpp/rclcpp.hpp>

// #include <pcl/point_types.h>
// #include <pcl/conversions.h>
// #include <pcl/io/pcd_io.h>
// #include <pcl/point_cloud.h>

#include <pcl_conversions/pcl_conversions.h>

#include <sensor_msgs/msg/point_cloud2.hpp>

using namespace std::chrono;
using namespace std::chrono_literals;

class FakeMapPublisher : public rclcpp::Node
{
public:
  FakeMapPublisher()
  : Node("fake_map_publisher")
  { 
    /* Parameters */
    std::string param_ns = "fake_map";
    this->declare_parameter(param_ns+".pcd_filepath", "");
    this->declare_parameter(param_ns+".frame_id", "world");
    this->declare_parameter(param_ns+".publishing_frequency", 1.0);

    pcd_filepath_ = this->get_parameter(param_ns+".pcd_filepath").as_string();
    frame_id_ = this->get_parameter(param_ns+".frame_id").as_string();
    pub_freq_ = this->get_parameter(param_ns+".publishing_frequency").as_double();

    RCLCPP_INFO(this->get_logger(), "Params: pcd_filepath(%s), frame_id(%s), pub_freq(%f)", 
                pcd_filepath_.c_str(), frame_id_.c_str(), pub_freq_);

    /* Publisher */
    fake_map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/fake_map", 10);

    cloud_msg_ = std::make_shared<sensor_msgs::msg::PointCloud2>();

    /* Load point cloud file */
    RCLCPP_INFO(this->get_logger(), "Loading pcd file path: %s", pcd_filepath_.c_str());
    if (pcl::io::loadPCDFile(pcd_filepath_, *cloud_msg_) == -1) 
    {
      RCLCPP_ERROR(this->get_logger(), "Invalid pcd file! Shutting down!");
      rclcpp::shutdown();
    }

    cloud_msg_->header.stamp = this->get_clock()->now();
    cloud_msg_->header.frame_id = frame_id_;
    cloud_msg_->is_dense = true;

    pub_fake_map_timer_ = this->create_wall_timer((1.0/pub_freq_) *1000ms, 
                                          std::bind(&FakeMapPublisher::pubFakeMapTimerCB, this));
  }

  void pubFakeMapTimerCB(){
    // std::cout << "cloud_msg_->points.size(): " << cloud_msg_->data.size() << std::endl;
    // std::cout << "cloud_msg_->height: " << cloud_msg_->height << std::endl;
    // std::cout << "cloud_msg_->width: " << cloud_msg_->width << std::endl;
    // std::cout << "cloud_msg_->is_dense: " << cloud_msg_->is_dense << std::endl;
    // RCLCPP_INFO(this->get_logger(), "Publishing fake point cloud map!");

    fake_map_pub_->publish(*cloud_msg_);
  }

private:
  std::string pcd_filepath_; // Path to pcd file
  std::string frame_id_;  // Frame id of map
  double pub_freq_;       // [Hz] map publishing frequency

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr fake_map_pub_; // Publisher of fake map

  rclcpp::TimerBase::SharedPtr pub_fake_map_timer_;	    // Timer for publishing fake map

  std::shared_ptr<sensor_msgs::msg::PointCloud2> cloud_msg_;   // Stored message of map for publishing
};

#endif  // FAKE_MAP_PUBLISHER_HPP_
