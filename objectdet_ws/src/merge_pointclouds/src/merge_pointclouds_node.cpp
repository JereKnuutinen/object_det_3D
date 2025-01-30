

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
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <pcl/filters/approximate_voxel_grid.h>



class MergePcNode : public rclcpp::Node
{
public:
    MergePcNode() : Node("merge_pointclouds_node"), tf_buffer_(std::make_shared<tf2_ros::Buffer>(this->get_clock())),
                    tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_))
    {
        GNSS_T_left = Eigen::Matrix4f::Identity();
        GNSS_T_left <<  0.9151, -0.2635, 0.27, 0.98,
                        0.2466,  0.9644, 0.0955, 0.66,
                        -0.2855,  -0.0219,  0.9581, -0.17,
                         0.0,      0.0,     0.0,     1.0;

        Left_T_right = Eigen::Matrix4f::Identity();
        Left_T_right <<  0.8896,  0.4567, -0.0088, -0.35,
                        -0.4482,  0.8690, -0.2096, -1.30,
                        -0.0880,  0.1904,  0.9778, -0.14,
                         0.0,      0.0,     0.0,     1.0;

        right_pc_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/synced_pc_lidar_1", 10, std::bind(&MergePcNode::pointCloudCallback, this, std::placeholders::_1));

        left_pc_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/synced_pc_lidar_2", 10, std::bind(&MergePcNode::pointCloud2Callback, this, std::placeholders::_1));

        global_pc_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/merged_global_pointcloud", 1);

        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&MergePcNode::mergePointClouds, this));
    }

private:
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        pcl::fromROSMsg(*msg, *pc1_);
        pc1_received_ = true;
    }

    void pointCloud2Callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        pcl::fromROSMsg(*msg, *pc2_);
        pc2_received_ = true;
    }

    void mergePointClouds()
    {
        if (!pc1_received_ || !pc2_received_) return;

        // Transform right LiDAR point cloud to left LiDAR frame
        pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_pc1(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::transformPointCloud(*pc1_, *transformed_pc1, Left_T_right);

        // Merge point clouds
        pcl::PointCloud<pcl::PointXYZ>::Ptr merged_pc(new pcl::PointCloud<pcl::PointXYZ>());
        *merged_pc = *pc2_ + *transformed_pc1;
        pcl::ApproximateVoxelGrid<pcl::PointXYZ> voxel;
        voxel.setInputCloud(merged_pc);
        voxel.setLeafSize(0.5f, 0.5f, 0.5f);  // More aggressive downsampling
        voxel.filter(*merged_pc);

        // Transform merged LiDAR point cloud to GNSS/IMU frame
        pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_pc_gnss(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::transformPointCloud(*merged_pc, *transformed_pc_gnss, GNSS_T_left);

        // Transform merged point cloud to the global frame
        pcl::PointCloud<pcl::PointXYZ>::Ptr global_pc(new pcl::PointCloud<pcl::PointXYZ>());
        if (!transformToGlobalFrame(transformed_pc_gnss, global_pc))
        {
            RCLCPP_WARN(this->get_logger(), "Failed to transform point cloud to global frame.");
            return;
        }

        // Accumulate point clouds in the global frame
        *global_merged_pc_ += *global_pc;
        // Set up the Approximate Voxel Grid filter
        // pcl::ApproximateVoxelGrid<pcl::PointXYZ> voxel2;
        // voxel2.setInputCloud(global_merged_pc_);
        // voxel2.setLeafSize(0.05f, 0.05f, 0.05f);  // Adjust leaf size as needed
        // voxel2.filter(*global_merged_pc_);

        // Publish the accumulated global point cloud
        sensor_msgs::msg::PointCloud2 output_msg;
        pcl::toROSMsg(*global_merged_pc_, output_msg);
        output_msg.header.frame_id = "map";
        output_msg.header.stamp = this->now();
        global_pc_pub_->publish(output_msg);

        RCLCPP_INFO(this->get_logger(), "Published merged global point cloud.");
    }

    bool transformToGlobalFrame(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input, pcl::PointCloud<pcl::PointXYZ>::Ptr &output)
    {
        geometry_msgs::msg::TransformStamped transform_stamped;
        try
        {
            transform_stamped = tf_buffer_->lookupTransform("map", "gnss_pose", tf2::TimePointZero);
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "Could not transform to global frame: %s", ex.what());
            return false;
        }

        // Extract translation
        Eigen::Vector3f translation(
            transform_stamped.transform.translation.x,
            transform_stamped.transform.translation.y,
            transform_stamped.transform.translation.z);

        // Extract rotation (as a quaternion)
        Eigen::Quaternionf rotation(
            transform_stamped.transform.rotation.w,
            transform_stamped.transform.rotation.x,
            transform_stamped.transform.rotation.y,
            transform_stamped.transform.rotation.z);

        // Convert to transformation matrix
        Eigen::Matrix4f global_transform_matrix = Eigen::Matrix4f::Identity();
        global_transform_matrix.block<3, 3>(0, 0) = rotation.toRotationMatrix();
        global_transform_matrix.block<3, 1>(0, 3) = translation;

        pcl::transformPointCloud(*input, *output, global_transform_matrix);
        return true;
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr right_pc_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr left_pc_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr global_pc_pub_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    pcl::PointCloud<pcl::PointXYZ>::Ptr pc1_{new pcl::PointCloud<pcl::PointXYZ>()};
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc2_{new pcl::PointCloud<pcl::PointXYZ>()};
    pcl::PointCloud<pcl::PointXYZ>::Ptr global_merged_pc_{new pcl::PointCloud<pcl::PointXYZ>()};

    bool pc1_received_ = false;
    bool pc2_received_ = false;

    rclcpp::TimerBase::SharedPtr timer_;
    Eigen::Matrix4f Left_T_right;
    Eigen::Matrix4f GNSS_T_left;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MergePcNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

// #include "rclcpp/rclcpp.hpp"
// #include <iostream>
// #include <memory>
// #include <pcl/point_cloud.h>
// #include <pcl/point_types.h>
// #include <pcl/common/transforms.h>
// #include <pcl_conversions/pcl_conversions.h>
// #include <sensor_msgs/msg/point_cloud2.hpp>
// #include <pcl/filters/filter.h>
// #include "pcl_ros/transforms.hpp"
// #include <Eigen/Dense>
// #include <tf2_ros/transform_listener.h>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.h>
// #include <chrono>

// class MergePcNode : public rclcpp::Node
// {
// public:
//     MergePcNode() : Node("merge_pointclouds_node")
//     {
    
// 		Left_T_right = Eigen::Matrix4f::Identity(); // put here Lidar2_T_lidar1 transformation

// 		// first row
// 		Left_T_right(0,0) = 0.8896;
// 		Left_T_right(0,1) = 0.4567;
// 		Left_T_right(0,2) = -0.0088;
// 		Left_T_right(0, 3) = -0.35; 

// 		// second row
// 		Left_T_right(1,0) = -0.4482;
// 		Left_T_right(1,1) = 0.8690;
// 		Left_T_right(1,2) =  -0.2096;
// 		Left_T_right(1, 3) = -1.30;

// 		// third row
// 		Left_T_right(2,0) = -0.088;
// 		Left_T_right(2,1) = 0.1904;
// 		Left_T_right(2,2) = 0.9778;
// 		Left_T_right(2, 3) = -0.14;

//         // Initialize subscribers, publishers
//         Right_pointcloud_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
//             "/synced_pc_lidar_1", 10, std::bind(&MergePcNode::pointCloudCallback, this, std::placeholders::_1)); // right lidar subsriber
        
//         Left_pointcloud_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
//             "/synced_pc_lidar_2", 10, std::bind(&MergePcNode::pointCloud2Callback, this, std::placeholders::_1)); // left lidar subsriber

//         pointcloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/merged_pointcloud", 10); // pusblish merged pc

        
//         // Create a timer that triggers every 0.1 seconds (100 ms)
// 	    timer_ = this->create_wall_timer(std::chrono::milliseconds(100),std::bind(&MergePcNode::mergePointClouds, this));
//     }

// private:
//     void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
//     {
//         // Convert sensor_msgs/PointCloud2 to PCL PointCloud
//         pcl::fromROSMsg(*msg, *pc1_);

//         // boolean flag that indicates that the pc1 is received
// 		pc1_received_ = true;
	
//     }

//     void pointCloud2Callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
//     {
//         // Convert sensor_msgs/PointCloud2 to PCL PointCloud
//         pcl::fromROSMsg(*msg, *pc2_);
        
// 		// boolean flag that indicates that the pc2 is received
//         pc2_received_ = true;

//     }

//     void mergePointClouds()
//     {
// 		if ((pc1_received_ == true) && (pc2_received_ == true)) {
// 			// Apply a transformation to the right lidar point cloud
// 		    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud{new pcl::PointCloud<pcl::PointXYZ>()};
// 		    pcl::transformPointCloud(*pc1_, *transformed_cloud, Left_T_right);

// 		    // Merge the point clouds from both
// 		    pcl::PointCloud<pcl::PointXYZ>::Ptr merged_cloud{new pcl::PointCloud<pcl::PointXYZ>()};
// 		    *merged_cloud = *pc2_ + *transformed_cloud;

// 		    // Publish the merged point cloud
// 		    sensor_msgs::msg::PointCloud2 output_msg;
// 		    pcl::toROSMsg(*merged_cloud, output_msg);
// 		    output_msg.header.frame_id = "map";
// 		    output_msg.header.stamp = this->now();
// 		    pointcloud_publisher_->publish(output_msg);
		    
// 		    RCLCPP_INFO(this->get_logger(), "Merged point clouds and published.");
// 		}
//     }

//     rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr Right_pointcloud_subscriber_;
//     rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr Left_pointcloud_subscriber_;
//     rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_publisher_;

//     std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
//     std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

//     pcl::PointCloud<pcl::PointXYZ>::Ptr pc1_{new pcl::PointCloud<pcl::PointXYZ>()};
//     pcl::PointCloud<pcl::PointXYZ>::Ptr pc2_{new pcl::PointCloud<pcl::PointXYZ>()};
//     pcl::PointCloud<pcl::PointXYZ>::Ptr pc2_transformed_{new pcl::PointCloud<pcl::PointXYZ>()};
//     pcl::PointCloud<pcl::PointXYZ>::Ptr merged_pc_{new pcl::PointCloud<pcl::PointXYZ>()};

//     bool pc1_received_ = false;
//     bool pc2_received_ = false;
    
//     rclcpp::TimerBase::SharedPtr timer_;
//     Eigen::Matrix4f Left_T_right;
// };

// int main(int argc, char **argv)
// {
//     rclcpp::init(argc, argv);

//     auto node = std::make_shared<MergePcNode>();
//     rclcpp::spin(node);
//     rclcpp::shutdown();

//     return 0;
// }

