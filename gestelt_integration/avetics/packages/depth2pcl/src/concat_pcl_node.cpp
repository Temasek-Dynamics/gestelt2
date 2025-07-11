#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "rclcpp/rclcpp.hpp"

#include <cstdio>
#include <unordered_map>
#include <mutex>

class ConcatPCL : public rclcpp::Node
{
public:
    ConcatPCL() : Node("concat_pcl_node")
    {
        const auto concat_pcl_topic = this->declare_parameter("concat_pcl_topic", std::string("/point_cloud/concat"));

        //create publisher
        pcl_concat_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            concat_pcl_topic,
            rclcpp::SensorDataQoS()
        );

        RCLCPP_INFO(this->get_logger(), "[Publish] %s", concat_pcl_topic.c_str());

        const uint16_t concat_pub_interval = this->declare_parameter("concat_pub_interval", 62); //in ms

        //callbacks in parallel
        auto reentrant_callback_group = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

        pcl_concat_pub_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(concat_pub_interval),
            std::bind(&ConcatPCL::timerCallback, this),
            reentrant_callback_group
        );

        rclcpp::SubscriptionOptions subscription_options;
        subscription_options.callback_group = reentrant_callback_group;

        //create subscribers
        pcl_topics_ = this->declare_parameter("pcl_topics", std::vector<std::string>());

        for (const auto& current_topic : pcl_topics_)
        {
            pcl_subscribers_.emplace_back(
                this->create_subscription<sensor_msgs::msg::PointCloud2>(
                    current_topic,
                    rclcpp::SensorDataQoS(),
                    [&](const sensor_msgs::msg::PointCloud2 &pcl_message)
                    {
                        pclCallback(pcl_message, current_topic);
                    },
                    // same as above but doesn't compile due to some ros2 bug but i forgot where i read it from
                    // std::bind(&ConcatPCL::pclCallback, this, std::placeholders::_1, current_topic)
                    subscription_options
                )
            );

            RCLCPP_INFO(this->get_logger(), "[Subscribe] %s", current_topic.c_str());
        } //for
    }

private:
    //subscribe
    std::vector<std::string> pcl_topics_;
    std::vector<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr> pcl_subscribers_;
    std::unordered_map<std::string, sensor_msgs::msg::PointCloud2> pcl_messages_;
    std::mutex pcl_messages_mutex_;

    //publish
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_concat_pub_;
    rclcpp::TimerBase::SharedPtr pcl_concat_pub_timer_;

    void timerCallback()
    {
        std::lock_guard<std::mutex> lock(pcl_messages_mutex_);

        if (pcl_messages_.empty())
            return;

        //https://github.com/ravijo/multiple_kinect_baxter_calibration/blob/5b883da744799abbf78f400f40e142d8450a7c9d/src/merge_pc.cpp#L152

        sensor_msgs::msg::PointCloud2 pcl_concat_msg;

        for (const auto& [current_pcl_topic, current_pcl_message] : pcl_messages_)
        {
            //Detect and copy the first message
            if (pcl_concat_msg.data.empty())
            {
                pcl_concat_msg = current_pcl_message;
                continue;
            }

            //assume stl knows best instead of manually reserving and doing memcpy
            pcl_concat_msg.data.insert(
                pcl_concat_msg.data.end(),
                current_pcl_message.data.begin(),
                current_pcl_message.data.end()
            );

            pcl_concat_msg.width += current_pcl_message.width;

            if (!current_pcl_message.is_dense)
                pcl_concat_msg.is_dense = false; //set to false if any of the message is false
        } //for

        pcl_concat_pub_->publish(pcl_concat_msg);
    }

    void pclCallback(const sensor_msgs::msg::PointCloud2 &pcl_message, const std::string& pcl_topic_)
    {
        std::lock_guard<std::mutex> lock(pcl_messages_mutex_);
        pcl_messages_[pcl_topic_] = pcl_message;
    }
}; //class ConcatPCL

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto concat_pcl_node = std::make_shared<ConcatPCL>();

    rclcpp::executors::MultiThreadedExecutor executor;

    executor.add_node(concat_pcl_node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}