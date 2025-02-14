#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl_conversions/pcl_conversions.h>
#include <cmath>

class WallDetection : public rclcpp::Node
{
public:
    WallDetection()
        : Node("wall_detection_node")
    {
        // 订阅激光雷达数据
        laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&WallDetection::laserCallback, this, std::placeholders::_1));

        // 发布检测到的直线
        wall_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/detected_walls", 10);
    }

private:
    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        // 将LaserScan数据转换为PCL点云
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        for (size_t i = 0; i < msg->ranges.size(); ++i)
        {
            float angle = msg->angle_min + i * msg->angle_increment;
            pcl::PointXYZ point;
            point.x = msg->ranges[i] * cos(angle);
            point.y = msg->ranges[i] * sin(angle);
            point.z = 0.0;
            cloud->points.push_back(point);
        }

        // 使用RANSAC进行直线检测
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_LINE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(0.01);
        seg.setInputCloud(cloud);
        seg.segment(*inliers, *coefficients);

        // 发布检测到的直线
        if (inliers->indices.size() > 0)
        {
            geometry_msgs::msg::PoseArray wall_poses;
            wall_poses.header.stamp = this->now();
            wall_poses.header.frame_id = "laser_frame";

            geometry_msgs::msg::Pose pose;
            pose.position.x = coefficients->values[0];
            pose.position.y = coefficients->values[1];
            pose.position.z = 0.0;
            pose.orientation.w = 1.0;
            wall_poses.poses.push_back(pose);

            wall_pub_->publish(wall_poses);

            // 计算直线的方向角
            double a = coefficients->values[3]; // 直线方程中的a
            double b = coefficients->values[4]; // 直线方程中的b
            double theta = (atan2(b, a) * 180) / M_PI; // 计算方向角

            RCLCPP_INFO(this->get_logger(), "Detected wall direction angle: %f radians", theta);
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr wall_pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WallDetection>());
    rclcpp::shutdown();
    return 0;
}