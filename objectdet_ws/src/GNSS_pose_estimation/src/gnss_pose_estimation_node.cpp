#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <Eigen/Dense>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <chrono>
#include "WGS84toCartesian.hpp"
#include <array>
#include <vector>
#include <visualization_msgs/msg/marker.hpp>
#include <math.h>
class GNSSPoseEstimationNode : public rclcpp::Node
{
public:
    GNSSPoseEstimationNode() : Node("gnss_pose_estimation_node")
    {
        gnss_subscriber_ = this->create_subscription<sensor_msgs::msg::NavSatFix>("/GNSS_messages_serial", 1, std::bind(&GNSSPoseEstimationNode::GNSSCallback, this, std::placeholders::_1));
        imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>("/Imu_messages", 10, std::bind(&GNSSPoseEstimationNode::ImuCallback, this, std::placeholders::_1));
        gnss_pose_publisher_ = this->create_publisher<geometry_msgs::msg::Pose>("/GNSS_pose_enu", 1);
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        first_time_here_ = true;
        gnss_received_ = false;
        imu_received_ = false;
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
                                         std::bind(&GNSSPoseEstimationNode::mergeGnssImu, this));
    }

private:
    void GNSSCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
    {
        if (first_time_here_ == true) {
            lat0_ = msg->latitude;
            lon0_ =  msg->longitude;
            first_time_here_ = false;
        }

        std::array<double, 2> cartesianPosition = wgs84::toCartesian({lat0_, lon0_}, {msg->latitude, msg->longitude});
        local_enu_x_ = cartesianPosition.at(0);
        local_enu_y_ = cartesianPosition.at(1);
        gnss_received_ = true;

    }

    void ImuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        double yawtemp = -msg->orientation_covariance[1] + M_PI/2;
        if (yawtemp > M_PI) {
        	yawtemp = yawtemp - M_PI/2;
        }
        yaw_ = yawtemp;
        pitch_ =  -msg->orientation_covariance[2];
        roll_ =  msg->orientation_covariance[3];
        imu_received_ = true;
    }

    void mergeGnssImu()
    {
        if (!gnss_received_ || !imu_received_)
            return;


        // Publish the GNSS pose
        geometry_msgs::msg::Pose gnss_pose;
        gnss_pose.position.x = local_enu_x_;
        gnss_pose.position.y = local_enu_y_;
        gnss_pose.position.z = 0; //msg->altitude;

        // Convert yaw, pitch, roll to quaternion
        tf2::Quaternion q;
        q.setRPY(roll_, pitch_, yaw_);  // Roll, Pitch, Yaw order
        // Set the quaternion to the GNSS pose orientation
        gnss_pose.orientation.w = q.w();
        gnss_pose.orientation.x = q.x();
        gnss_pose.orientation.y = q.y();
        gnss_pose.orientation.z = q.z();

        gnss_pose_publisher_->publish(gnss_pose);
        RCLCPP_INFO(this->get_logger(), "GNSS pose former.");

        // Create a transform message
        geometry_msgs::msg::TransformStamped transform_stamped;
        transform_stamped.header.stamp = this->get_clock()->now();
        transform_stamped.header.frame_id = "map";  // Base frame, change if necessary
        transform_stamped.child_frame_id = "gnss_pose"; // GNSS pose frame

        // Set translation and orientation from the GNSS pose
        transform_stamped.transform.translation.x = gnss_pose.position.x;
        transform_stamped.transform.translation.y = gnss_pose.position.y;
        transform_stamped.transform.translation.z = gnss_pose.position.z;
        transform_stamped.transform.rotation = gnss_pose.orientation;

        // Broadcast the transform
        tf_broadcaster_->sendTransform(transform_stamped);

    }

	double lat0_;
    double lon0_;
    bool first_time_here_;
    bool gnss_received_, imu_received_;
    double local_enu_x_;
    double local_enu_y_;
    double yaw_;
    double roll_;
    double pitch_;

    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gnss_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;

    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr gnss_pose_publisher_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<GNSSPoseEstimationNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}

