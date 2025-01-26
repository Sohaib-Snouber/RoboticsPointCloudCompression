#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/byte_multi_array.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/compression/octree_pointcloud_compression.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sstream>
#include <chrono>

class OctreeDecompressionNode : public rclcpp::Node
{
public:
    OctreeDecompressionNode()
        : Node("octree_decompression_node")
    {
        // Subscription to compressed data
        compressed_data_subscriber_ = this->create_subscription<std_msgs::msg::ByteMultiArray>(
            "/points/compressed_octree", 10,
            std::bind(&OctreeDecompressionNode::compressedDataCallback, this, std::placeholders::_1));

        // Publisher for decompressed point cloud
        decompressed_cloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/points/decompressed_octree", 10);

        RCLCPP_INFO(this->get_logger(), "Octree Decompression Node Started!");
    }

private:
    void compressedDataCallback(const std_msgs::msg::ByteMultiArray::SharedPtr msg)
    {
        // Configure the Octree Compressor
        pcl::io::OctreePointCloudCompression<pcl::PointXYZRGBA> decompressor(
            pcl::io::HIGH_RES_ONLINE_COMPRESSION_WITH_COLOR, // Compression profile
            true,                                                // Show compression statistics
            0.001,                                             // Octree resolution (1 mm voxels)
            24,                                                 // Point coordinate bit budget (bits for XYZ precision)
            12                                                   // Color bit budget (bits for RGB precision)
        );

        auto decompression_start_time = std::chrono::high_resolution_clock::now();

        // Deserialize the compressed data
        std::stringstream compressed_data_stream;
        compressed_data_stream.write(reinterpret_cast<const char *>(msg->data.data()), msg->data.size());

        // Decompress the point cloud
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr decompressed_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>());
        decompressor.decodePointCloud(compressed_data_stream, decompressed_cloud);
        auto decompression_end_time = std::chrono::high_resolution_clock::now();

        // Calculate decompression time
        std::chrono::duration<double> decompression_time = (decompression_end_time - decompression_start_time)*1000;

        RCLCPP_INFO(this->get_logger(), "Decompression completed in %.3f ms", decompression_time.count());

        // Publish the decompressed cloud
        sensor_msgs::msg::PointCloud2 output_msg;
        pcl::toROSMsg(*decompressed_cloud, output_msg);
        output_msg.header.stamp = this->get_clock()->now();
        output_msg.header.frame_id = "map";
        decompressed_cloud_publisher_->publish(output_msg);

        RCLCPP_INFO(this->get_logger(), "Decompressed and published point cloud");
    }

    rclcpp::Subscription<std_msgs::msg::ByteMultiArray>::SharedPtr compressed_data_subscriber_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr decompressed_cloud_publisher_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OctreeDecompressionNode>());
    rclcpp::shutdown();
    return 0;
}

