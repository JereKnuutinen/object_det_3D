

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
        voxel.setLeafSize(0.3f, 0.3f, 0.3f);
    }

private:
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(pc_mutex_);
        pcl::fromROSMsg(*msg, *pc1_);
        pc1_received_ = true;
    }

    void pointCloud2Callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(pc_mutex_);
        pcl::fromROSMsg(*msg, *pc2_);
        pc2_received_ = true;
    }

    void mergePointClouds()
    {
        std::lock_guard<std::mutex> lock(pc_mutex_);
        if (pc1_received_ == false || pc2_received_ == false)  {
            return;
        }

        // Transform right LiDAR to left LiDAR frame in-place
        pcl::transformPointCloud(*pc1_, *pc1_, Left_T_right);
        *pc2_ += *pc1_; // Merge into left frame

        // Apply voxel grid filter for downsampling
        voxel.setInputCloud(pc2_);
        voxel.filter(*pc2_);

        // Transform to GNSS frame in-place
        pcl::transformPointCloud(*pc2_, *pc2_, GNSS_T_left);

        // Transform to global frame if possible
        if (transformToGlobalFrame(pc2_, pc2_))
        {
            *global_merged_pc_ += *pc2_;

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

    //pcl::ApproximateVoxelGrid<pcl::PointXYZ> voxel;
    pcl::VoxelGrid<pcl::PointXYZ> voxel;

    std::mutex pc_mutex_;  // Protects shared point cloud data
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

