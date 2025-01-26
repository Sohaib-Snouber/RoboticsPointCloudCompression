#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl_conversions/pcl_conversions.h>

class SavePointCloudNode : public rclcpp::Node
{
public:
    SavePointCloudNode()
        : Node("save_point_cloud_node")
    {
        // Declare a parameter for the topic name
        this->declare_parameter<std::string>("topic_name", "/points/compressed_voxel");
        this->declare_parameter<std::string>("output_file", "voxel_compressed.ply");

        // Get the topic name and output file from parameters
        topic_name_ = this->get_parameter("topic_name").as_string();
        output_file_ = this->get_parameter("output_file").as_string();

        // Subscription to the user-defined topic
        point_cloud_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            topic_name_, 10, std::bind(&SavePointCloudNode::pointCloudCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Waiting for a point cloud on topic: %s", topic_name_.c_str());
    }

private:
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Point cloud received. Saving to file: %s", output_file_.c_str());

        // Convert ROS message to PCL PointCloud
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>());
        pcl::fromROSMsg(*msg, *cloud);

        // Save the point cloud to a .ply file
        if (pcl::io::savePLYFileBinary(output_file_, *cloud) == 0)
        {
            RCLCPP_INFO(this->get_logger(), "Point cloud saved successfully to: %s", output_file_.c_str());
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to save point cloud to file: %s", output_file_.c_str());
        }

        // Shutdown the node after saving
        rclcpp::shutdown();
    }

    std::string topic_name_;
    std::string output_file_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_subscriber_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SavePointCloudNode>());
    return 0;
}
