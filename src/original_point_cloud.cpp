#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <chrono>

class PointCloudSubscriberNode : public rclcpp::Node
{
public:
    PointCloudSubscriberNode()
        : Node("point_cloud_subscriber_node")
    {
        // Subscription to the point cloud topic
        point_cloud_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/points/xyzrgba", 10,
            std::bind(&PointCloudSubscriberNode::pointCloudCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Point Cloud Subscriber Node Started!");
    }

private:
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // Get the timestamp of the received message
        auto received_time = this->get_clock()->now();

        // Calculate the latency
        auto message_time = rclcpp::Time(msg->header.stamp);
        auto latency = received_time - message_time;

        RCLCPP_INFO(
            this->get_logger(),
            "Received point cloud. Communication latency: %.3f ms",
            latency.seconds() * 1000.0);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_subscriber_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudSubscriberNode>());
    rclcpp::shutdown();
    return 0;
}
