#include "rclcpp/rclcpp.hpp"
#include <iostream>
#include <memory>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/filters/filter.h>
#include "pcl_ros/transforms.hpp"
#include <Eigen/Dense>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <chrono>

class MergePcNode : public rclcpp::Node
{
public:
    MergePcNode() : Node("merge_pointclouds_node")
    {
    
		Left_T_right = Eigen::Matrix4f::Identity(); // put here Lidar2_T_lidar1 transformation

		// first row
		Left_T_right(0,0) = 0.8896;
		Left_T_right(0,1) = 0.4567;
		Left_T_right(0,2) = -0.0088;
		Left_T_right(0, 3) = -0.35; 

		// second row
		Left_T_right(1,0) = -0.4482;
		Left_T_right(1,1) = 0.8690;
		Left_T_right(1,2) =  -0.2096;
		Left_T_right(1, 3) = -1.30;

		// third row
		Left_T_right(2,0) = -0.088;
		Left_T_right(2,1) = 0.1904;
		Left_T_right(2,2) = 0.9778;
		Left_T_right(2, 3) = -0.14;

        // Initialize subscribers, publishers
        Right_pointcloud_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/synced_pc_lidar_1", 10, std::bind(&MergePcNode::pointCloudCallback, this, std::placeholders::_1)); // right lidar subsriber
        
        Left_pointcloud_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/synced_pc_lidar_2", 10, std::bind(&MergePcNode::pointCloud2Callback, this, std::placeholders::_1)); // left lidar subsriber

        pointcloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/merged_pointcloud", 10); // pusblish merged pc

        
        // Create a timer that triggers every 0.1 seconds (100 ms)
	    timer_ = this->create_wall_timer(std::chrono::milliseconds(100),std::bind(&MergePcNode::mergePointClouds, this));
    }

private:
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // Convert sensor_msgs/PointCloud2 to PCL PointCloud
        pcl::fromROSMsg(*msg, *pc1_);

        // boolean flag that indicates that the pc1 is received
		pc1_received_ = true;
	
    }

    void pointCloud2Callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // Convert sensor_msgs/PointCloud2 to PCL PointCloud
        pcl::fromROSMsg(*msg, *pc2_);
        
		// boolean flag that indicates that the pc2 is received
        pc2_received_ = true;

    }

    void mergePointClouds()
    {
		if ((pc1_received_ == true) && (pc2_received_ == true)) {
			// Apply a transformation to the right lidar point cloud
		    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud{new pcl::PointCloud<pcl::PointXYZ>()};
		    pcl::transformPointCloud(*pc1_, *transformed_cloud, Left_T_right);

		    // Merge the point clouds from both
		    pcl::PointCloud<pcl::PointXYZ>::Ptr merged_cloud{new pcl::PointCloud<pcl::PointXYZ>()};
		    *merged_cloud = *pc2_ + *transformed_cloud;

		    // Publish the merged point cloud
		    sensor_msgs::msg::PointCloud2 output_msg;
		    pcl::toROSMsg(*merged_cloud, output_msg);
		    output_msg.header.frame_id = "map";
		    output_msg.header.stamp = this->now();
		    pointcloud_publisher_->publish(output_msg);
		    
		    RCLCPP_INFO(this->get_logger(), "Merged point clouds and published.");
		}
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr Right_pointcloud_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr Left_pointcloud_subscriber_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_publisher_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    pcl::PointCloud<pcl::PointXYZ>::Ptr pc1_{new pcl::PointCloud<pcl::PointXYZ>()};
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc2_{new pcl::PointCloud<pcl::PointXYZ>()};
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc2_transformed_{new pcl::PointCloud<pcl::PointXYZ>()};
    pcl::PointCloud<pcl::PointXYZ>::Ptr merged_pc_{new pcl::PointCloud<pcl::PointXYZ>()};

    bool pc1_received_ = false;
    bool pc2_received_ = false;
    
    rclcpp::TimerBase::SharedPtr timer_;
    Eigen::Matrix4f Left_T_right;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<MergePcNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}

