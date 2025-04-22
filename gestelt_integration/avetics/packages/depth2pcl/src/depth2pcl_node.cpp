#include <cstdio>

#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl_ros/transforms.hpp>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include <tf2_ros/transform_listener.h>

class DepthToPCLConverter : public rclcpp::Node
{
public:
    DepthToPCLConverter() : Node("depth2pcl_node")
    {
        tfBuffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tfListener = std::make_unique<tf2_ros::TransformListener>(*tfBuffer);

        depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/depth/rect",
            rclcpp::SensorDataQoS(),
            std::bind(&DepthToPCLConverter::depthCallback, this, std::placeholders::_1)
            );

        camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "/depth/camera_info",
            rclcpp::SensorDataQoS(),
            std::bind(&DepthToPCLConverter::cameraInfoCallback, this, std::placeholders::_1));

        pcl_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/point_cloud", rclcpp::SensorDataQoS());

        min_dist_ = this->declare_parameter("min_dist", 0.1);
        max_dist_ = this->declare_parameter("max_dist", 20.0);
        pcl_frame_ = this->declare_parameter("pcl_frame_id", std::string("odom")); //frame to publish the point cloud in

        downsample_leaf_size_ = this->declare_parameter("downsample_leaf_size", 0.01f);
        minimum_points_per_voxel_ = this->declare_parameter("minimum_points_per_voxel", 1);

        has_camera_info_ = false;
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_pub_;

    bool has_camera_info_;
    float fx_, fy_, cx_, cy_;
    double min_dist_, max_dist_;
    std::string transform_frame_;
    std::string pcl_frame_;

    float downsample_leaf_size_;
    int minimum_points_per_voxel_;

    // transform
    std::unique_ptr<tf2_ros::Buffer> tfBuffer;
    std::unique_ptr<tf2_ros::TransformListener> tfListener;

    void cameraInfoCallback(const sensor_msgs::msg::CameraInfo &camera_info)
    {
        fx_ = camera_info.k[0];
        fy_ = camera_info.k[4];
        cx_ = camera_info.k[2];
        cy_ = camera_info.k[5];
        has_camera_info_ = true;
    }

    void depthCallback(const sensor_msgs::msg::Image &depth_msg)
    {
        if (!has_camera_info_)
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for camera info...");
            return;
        }

        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_32FC1);
        }
        catch (cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        int width = cv_ptr->image.cols;
        int height = cv_ptr->image.rows;

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        cloud->header.frame_id = depth_msg.header.frame_id;
        cloud->width = width;
        cloud->height = height;
        cloud->is_dense = false;
        cloud->points.resize(width * height);

        for (int v = 0; v < height; ++v)
        {
            for (int u = 0; u < width; ++u)
            {
                float depth_value = cv_ptr->image.at<float>(v, u);

                if (std::isnan(depth_value) || depth_value <= 0.0 || depth_value < min_dist_ || depth_value > max_dist_)
                {
                    depth_value = max_dist_; // Set depth to max_dist_ to raycast and clear all empty space in between

                    // cloud->points[v * width + u].x = std::numeric_limits<float>::quiet_NaN();
                    // cloud->points[v * width + u].y = std::numeric_limits<float>::quiet_NaN();
                    // cloud->points[v * width + u].z = std::numeric_limits<float>::quiet_NaN();
                    // continue;
                }

                float z = depth_value;
                float x = (u - cx_) * z / fx_;
                float y = (v - cy_) * z / fy_;

                cloud->points[v * width + u].x = x;
                cloud->points[v * width + u].y = y;
                cloud->points[v * width + u].z = z;
            }
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr filterInput(new pcl::PointCloud<pcl::PointXYZ>);

        if (depth_msg.header.frame_id != pcl_frame_)
        {
            // Apply transformation
            try
            {
                RCLCPP_INFO_ONCE(this->get_logger(), "[depthCallback] Transforming frame %s to %s", depth_msg.header.frame_id.c_str(), pcl_frame_.c_str());
                pcl_ros::transformPointCloud<pcl::PointXYZ>(pcl_frame_, *cloud, *filterInput, *tfBuffer);
            }
            catch (std::runtime_error &ex)
            {
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "[depthCallback] %s", ex.what());
                return;
            }
        }
        else
        {
            filterInput = cloud;
        }

        // Downsample
        // https://pcl.readthedocs.io/projects/tutorials/en/master/voxel_grid.html
        pcl::PointCloud<pcl::PointXYZ> filterOutput;

        pcl::VoxelGrid<pcl::PointXYZ> sor;
        sor.setInputCloud(filterInput);
        sor.setLeafSize(downsample_leaf_size_, downsample_leaf_size_, downsample_leaf_size_);
        sor.setMinimumPointsNumberPerVoxel(minimum_points_per_voxel_);
        sor.filter(filterOutput);

        /*
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>(filterOutput));
        pcl::PointCloud<pcl::PointXYZ> filterOutput2;

        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor2;
        sor2.setInputCloud(cloud2);
        sor2.setMeanK(50);
        sor2.setStddevMulThresh(1.0);
        sor2.filter(filterOutput2);
        */

        // Convert to ROS message and publish
        sensor_msgs::msg::PointCloud2 output;
        pcl::toROSMsg(filterOutput, output); //downsample
        // pcl::toROSMsg(*cloud, output); //don't downsample

        output.header = depth_msg.header;
        output.header.frame_id = pcl_frame_;

        pcl_pub_->publish(output);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DepthToPCLConverter>());
    rclcpp::shutdown();
    return 0;
}