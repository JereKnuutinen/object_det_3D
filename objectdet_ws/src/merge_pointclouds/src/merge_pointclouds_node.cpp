

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
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <mutex>
#include "geometry_msgs/msg/pose.hpp"

class MergePcNode : public rclcpp::Node
{
public:
    MergePcNode() : Node("merge_pointclouds_node")
    {

        rclcpp::QoS video_qos(10);
        video_qos.keep_last(10);
        video_qos.best_effort();
        video_qos.durability_volatile();

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

        // Initialize the transformation matrix as identity
        global_transform_matrix_ = Eigen::Matrix4f::Identity();

        // Subscribe to the GNSS pose in the map frame
        transform_sub_ = this->create_subscription<geometry_msgs::msg::Pose>(
            "/GNSS_pose_enu", video_qos, std::bind(&MergePcNode::poseCallback, this, std::placeholders::_1));


        right_pc_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/synced_pc_lidar_1", video_qos, std::bind(&MergePcNode::pointCloudCallback, this, std::placeholders::_1));

        left_pc_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/synced_pc_lidar_2", video_qos, std::bind(&MergePcNode::pointCloud2Callback, this, std::placeholders::_1));

        global_pc_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/merged_global_pointcloud", video_qos);

        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&MergePcNode::mergePointClouds, this));
        voxel.setLeafSize(0.5f, 0.5f, 0.5f);
    }

private:

    void poseCallback(const geometry_msgs::msg::Pose::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(data_mutex_);

        // Extract translation
        Eigen::Vector3f translation(msg->position.x, msg->position.y, msg->position.z);

        // Extract quaternion and convert to rotation matrix
        Eigen::Quaternionf rotation(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);

        // Construct the 4x4 transformation matrix
        global_transform_matrix_.setIdentity();
        global_transform_matrix_.block<3, 3>(0, 0) = rotation.toRotationMatrix();
        global_transform_matrix_.block<3, 1>(0, 3) = translation;
    }

    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(data_mutex_);

        pcl::fromROSMsg(*msg, *pc1_);
        pc1_received_ = true;
    }

    void pointCloud2Callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        pcl::fromROSMsg(*msg, *pc2_);
        pc2_received_ = true;
    }

    void mergePointClouds()
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        if (!pc1_received_ || !pc2_received_) {
            return;
        }

        // Create new point clouds for transformed and merged data
        pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_pc1(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr merged_pc(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_pc(new pcl::PointCloud<pcl::PointXYZ>);
        
        // Transform right LiDAR to left LiDAR frame and store in transformed_pc1
        pcl::transformPointCloud(*pc1_, *transformed_pc1, Left_T_right);
        
        // Merge into left frame and store in merged_pc
        *merged_pc += *transformed_pc1;
        *merged_pc += *pc2_; // Assuming pc2_ is in the left frame already

        // Apply voxel grid filter for downsampling and store in filtered_pc
        voxel.setInputCloud(merged_pc);
        voxel.filter(*filtered_pc);

        // Transform filtered point cloud to GNSS frame
        pcl::PointCloud<pcl::PointXYZ>::Ptr gnss_transformed_pc(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::transformPointCloud(*filtered_pc, *gnss_transformed_pc, GNSS_T_left);

        // Transform to global frame if possible
        if (transformToGlobalFrame(gnss_transformed_pc, gnss_transformed_pc))
        {
            // Merge into global merged point cloud
            *global_merged_pc_ += *gnss_transformed_pc;

            // Publish
            sensor_msgs::msg::PointCloud2 output_msg;
            pcl::toROSMsg(*global_merged_pc_, output_msg);
            output_msg.header.frame_id = "map";
            output_msg.header.stamp = this->now();
            global_pc_pub_->publish(output_msg);
        }
    }


    bool transformToGlobalFrame(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input, pcl::PointCloud<pcl::PointXYZ>::Ptr &output)
    {
        pcl::transformPointCloud(*input, *output, global_transform_matrix_);
        return true;
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr right_pc_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr left_pc_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr transform_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr global_pc_pub_;

    // Point cloud storage
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc1_{new pcl::PointCloud<pcl::PointXYZ>()};
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc2_{new pcl::PointCloud<pcl::PointXYZ>()};
    pcl::PointCloud<pcl::PointXYZ>::Ptr global_merged_pc_{new pcl::PointCloud<pcl::PointXYZ>()};

    bool pc1_received_ = false;
    bool pc2_received_ = false;

    // Timer for processing
    rclcpp::TimerBase::SharedPtr timer_;

    // Transformation matrices
    Eigen::Matrix4f Left_T_right;
    Eigen::Matrix4f GNSS_T_left;
    Eigen::Matrix4f global_transform_matrix_;

    // Voxel grid filter
    pcl::VoxelGrid<pcl::PointXYZ> voxel;

    // Mutexes for thread safety
    std::mutex data_mutex_;  // Protects point cloud data and transformation matrix

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MergePcNode>();
    // rclcpp::spin(node);
    // Multi-threaded executor with default number of threads
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin(); // Runs callbacks concurrently

    rclcpp::shutdown();
    return 0;
}

