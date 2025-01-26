#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <chrono>

class VoxelGridDecompressionNode : public rclcpp::Node
{
public:
    VoxelGridDecompressionNode()
        : Node("voxel_grid_decompression_node")
    {
        compressed_cloud_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/points/compressed_voxel", 10,
            std::bind(&VoxelGridDecompressionNode::compressedCloudCallback, this, std::placeholders::_1));

        decompressed_cloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/points/decompressed_voxel", 10);

        RCLCPP_INFO(this->get_logger(), "Voxel Grid Decompression Node Started!");
    }

private:
    void compressedCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr decompressed_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>());
        pcl::fromROSMsg(*msg, *decompressed_cloud);

        // Measure communication time
        auto now = this->now();
        auto published_time = msg->header.stamp;
        auto communication_duration = (now - published_time).seconds();

        RCLCPP_INFO(this->get_logger(), "Communication delay: %.6f seconds", communication_duration);

        sensor_msgs::msg::PointCloud2 output_msg;
        pcl::toROSMsg(*decompressed_cloud, output_msg);
        output_msg.header = msg->header;
        decompressed_cloud_publisher_->publish(output_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr compressed_cloud_subscriber_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr decompressed_cloud_publisher_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VoxelGridDecompressionNode>());
    rclcpp::shutdown();
    return 0;
}
