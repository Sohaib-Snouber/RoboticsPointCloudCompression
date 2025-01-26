#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/byte_multi_array.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/compression/octree_pointcloud_compression.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sstream>
#include <chrono>
#include <fstream>

class OctreeCompressionNode : public rclcpp::Node
{
public:
    OctreeCompressionNode()
        : Node("octree_compression_node")
    {
        // Subscription to input point cloud
        point_cloud_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/points/xyzrgba", 10,
            std::bind(&OctreeCompressionNode::pointCloudCallback, this, std::placeholders::_1));

        // Publisher for compressed data
        compressed_data_publisher_ = this->create_publisher<std_msgs::msg::ByteMultiArray>(
            "/points/compressed_octree", 10);

        RCLCPP_INFO(this->get_logger(), "Octree Compression Node Started!");
    }

private:
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // Convert ROS message to PCL PointCloud
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>());
        pcl::fromROSMsg(*msg, *cloud);

        // Configure the Octree Compressor
        pcl::io::OctreePointCloudCompression<pcl::PointXYZRGBA> compressor(
            pcl::io::LOW_RES_ONLINE_COMPRESSION_WITH_COLOR, true);

        // Compress the point cloud
        std::stringstream compressed_data_stream;
        auto compression_start_time = std::chrono::high_resolution_clock::now();
        compressor.encodePointCloud(cloud, compressed_data_stream);
        auto compression_end_time = std::chrono::high_resolution_clock::now();

        // Save compressed data to calculate size
        std::ofstream compressed_file("octree_compressed_data.bin", std::ios::binary);
        compressed_file << compressed_data_stream.rdbuf();
        compressed_file.close();

        // Calculate compression time
        std::chrono::duration<double> compression_time = compression_end_time - compression_start_time;

        // Calculate compression ratio
        size_t original_size = cloud->size() * sizeof(pcl::PointXYZRGBA);
        std::ifstream file_stream("octree_compressed_data.bin", std::ios::binary | std::ios::ate);
        size_t compressed_size = file_stream.tellg();
        file_stream.close();
        double compression_ratio = (1.0 - static_cast<double>(compressed_size) / original_size) * 100;

        // Log the compression results
        RCLCPP_INFO(this->get_logger(), "Compression completed in %.3f seconds", compression_time.count());
        RCLCPP_INFO(this->get_logger(), "Original size: %zu bytes, Compressed size: %zu bytes", original_size, compressed_size);
        RCLCPP_INFO(this->get_logger(), "Compression ratio: %.2f%% reduction", compression_ratio);

        // Convert compressed data to ByteMultiArray
        std_msgs::msg::ByteMultiArray compressed_msg;
        compressed_msg.data = std::vector<uint8_t>(
            std::istreambuf_iterator<char>(compressed_data_stream),
            std::istreambuf_iterator<char>());

        compressed_data_publisher_->publish(compressed_msg);
        RCLCPP_INFO(this->get_logger(), "Published compressed data");
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_subscriber_;
    rclcpp::Publisher<std_msgs::msg::ByteMultiArray>::SharedPtr compressed_data_publisher_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OctreeCompressionNode>());
    rclcpp::shutdown();
    return 0;
}

