#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <chrono>

class VoxelGridFilteringNode : public rclcpp::Node
{
public:
    VoxelGridFilteringNode()
        : Node("voxel_grid_filtering_node")
    {
        // Subscription to input point cloud
        point_cloud_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/points/xyzrgba", 10,
            std::bind(&VoxelGridFilteringNode::pointCloudCallback, this, std::placeholders::_1));

        // Publisher for filtered point cloud
        compressed_cloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/points/compressed_voxel", 10);

        RCLCPP_INFO(this->get_logger(), "Voxel Grid Filtering Node Started!");
    }

private:
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // Convert ROS message to PCL PointCloud
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>());
        pcl::fromROSMsg(*msg, *cloud);

        // Start timer for compression
        auto compression_start = std::chrono::high_resolution_clock::now();

        // Apply Voxel Grid Filtering
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>());
        pcl::VoxelGrid<pcl::PointXYZRGBA> voxel_filter;
        voxel_filter.setInputCloud(cloud);
        voxel_filter.setLeafSize(0.0001f, 0.0001f, 0.0001f); // Adjust voxel size as needed
        voxel_filter.filter(*filtered_cloud);

        // Measure compression time
        auto compression_end = std::chrono::high_resolution_clock::now();
        auto compression_duration = std::chrono::duration_cast<std::chrono::milliseconds>(compression_end - compression_start).count();

        // Calculate compression ratio
        size_t original_size = cloud->size() * sizeof(pcl::PointXYZRGBA);
        size_t compressed_size = filtered_cloud->size() * sizeof(pcl::PointXYZRGBA);
        double compression_ratio = (1.0 - (static_cast<double>(compressed_size) / original_size))*100 ;

        RCLCPP_INFO(this->get_logger(), "Compression completed in %ld ms.", compression_duration);
        RCLCPP_INFO(this->get_logger(), "Original size: %zu bytes", original_size);
        RCLCPP_INFO(this->get_logger(), "Compressed size: %zu bytes", compressed_size);   
        RCLCPP_INFO(this->get_logger(), "Compression ratio: %.2f%% Reduction", compression_ratio);

        // Publish compressed point cloud
        sensor_msgs::msg::PointCloud2 output_msg;
        pcl::toROSMsg(*filtered_cloud, output_msg);

        output_msg.header.stamp = this->now();
        output_msg.header.frame_id = msg->header.frame_id;

        compressed_cloud_publisher_->publish(output_msg);

        RCLCPP_INFO(this->get_logger(), "Published compressed point cloud.");
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_subscriber_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr compressed_cloud_publisher_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VoxelGridFilteringNode>());
    rclcpp::shutdown();
    return 0;
}
