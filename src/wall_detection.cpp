#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <deque>
#include <numeric>
#include <algorithm>
#include <iostream>
#include <random>

#define MAX_RANGES 1.0
#define R_INF 30.0
using namespace std;
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
        wall_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/wall_pose", 10);

        // 初始化TF
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    }

private:
    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
    {
        int scan_count = scan->ranges.size();
        vector<float> ranges_filtered;
        ranges_filtered.resize(scan_count);
        // Preprocess laser ranges
        for (size_t i = 0; i < scan_count; i++) {
            double r = scan->ranges[i];
            if (r > MAX_RANGES || isinf(r) || std::isnan(r)) {//r > MAX_RANGES || 
                // std::random_device rd;          // 随机设备，用于种子
                // std::mt19937 gen(rd());         // Mersenne Twister引擎
                // std::uniform_real_distribution<double> dist(0.0, 1.0); // 生成[0,1)
                ranges_filtered[i] = R_INF;
            } else {
                ranges_filtered[i] = r;
            }
        }

        // Smooth laser ranges
        for (size_t i = 1; i < scan_count - 2; i++) {
            if (ranges_filtered[i] > R_INF - R_INF/100.0  && (fabs(ranges_filtered[i + 1] - ranges_filtered[i - 1]) < 0.03)) {
                ranges_filtered[i] = (ranges_filtered[i + 1] + ranges_filtered[i - 1]) / 2.0f;
            } else if (ranges_filtered[i] > R_INF - R_INF/100.0 && ranges_filtered[i + 1] > R_INF - R_INF/100.0 &&
                        (fabs(ranges_filtered[i + 2] - ranges_filtered[i - 1]) < 0.03)) {
                ranges_filtered[i] = ranges_filtered[i + 1] = (ranges_filtered[i + 2] + ranges_filtered[i - 1]) / 2.0f;
            }
        }
                
        // 将LaserScan数据转换为PCL点云
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        for (size_t i = 0; i < scan->ranges.size(); ++i)
        {
            float angle = scan->angle_min + i * scan->angle_increment;
            pcl::PointXYZ point;
            if(ranges_filtered[i] > R_INF - R_INF/100.0){
                continue;
            }
            point.x = ranges_filtered[i] * cos(angle);
            point.y = ranges_filtered[i] * sin(angle);
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

            // 计算直线的方向角
            double a = coefficients->values[3]; // 直线方程中的a
            double b = coefficients->values[4]; // 直线方程中的b
            // double theta = (atan2(b, a) * 180) / M_PI; // 计算方向角
            double theta = atan2(b, a) + M_PI/2; // 直线方向角逆时针旋转pi/2
            tf2::Quaternion q;
            q.setRPY(0, 0, theta); // 设置旋转角度
            geometry_msgs::msg::Quaternion wall_pose_quat;
            wall_pose_quat = tf2::toMsg(q);//将tf2::Quaternion转换成geometry_msgs::msg::Quaternion
            pose.orientation = wall_pose_quat;
            // double pose_dis = sqrt(pow(pose.position.x - 0, 2) + pow(pose.position.y - 0, 2));
            // RCLCPP_INFO(this->get_logger(), "Detected distance of wall pose to laser origin: %f", pose_dis);

            wall_poses.poses.push_back(pose);
            wall_pub_->publish(wall_poses);
            RCLCPP_INFO(this->get_logger(), "Detected wall direction angle: %f radians, and wall pose size: %d", 
                        (theta*180)/M_PI, inliers->indices.size());

            // 计算激光雷达原点到直线的距离
            double x1 = coefficients->values[0]; // 直线上的一个点
            double y1 = coefficients->values[1];
            double m = coefficients->values[4] / coefficients->values[3]; // 斜率
            double C = y1 - m * x1; // 计算C
            double distance = fabs(C) / sqrt(m * m + 1); // 计算距离
            RCLCPP_INFO(this->get_logger(), "Distance from laser origin to wall: %f meters", distance);

            // 发布TF变换
            geometry_msgs::msg::TransformStamped transformStamped;
            transformStamped.header.stamp = this->now();
            transformStamped.header.frame_id = "laser_link";
            transformStamped.child_frame_id = "wall_link";
            // 设置变换的平移部分
            transformStamped.transform.translation.x = coefficients->values[0];
            transformStamped.transform.translation.y = coefficients->values[1];
            transformStamped.transform.translation.z = 0.0;
            // 设置变换的旋转部分
            transformStamped.transform.rotation.x = q.x();
            transformStamped.transform.rotation.y = q.y();
            transformStamped.transform.rotation.z = q.z();
            transformStamped.transform.rotation.w = q.w();
            // 发布TF变换
            tf_broadcaster_->sendTransform(transformStamped);

            // // 记录最新的五个theta值
            // if (theta_values_.size() >= 5) {
            //     theta_values_.pop_front(); // 移除最早的theta值
            // }
            // theta_values_.push_back(theta);
            // // 计算均值和方差
            // if (theta_values_.size() >= 1) {
            //     double mean = std::accumulate(theta_values_.begin(), theta_values_.end(), 0.0) / theta_values_.size();
            //     double variance = 0.0;
            //     for (double t : theta_values_) {
            //         variance += (t - mean) * (t - mean);
            //     }
            //     variance /= theta_values_.size();

            //     RCLCPP_INFO(this->get_logger(), "Theta mean (last 5): %f degree", (mean*180)/M_PI);
            //     RCLCPP_INFO(this->get_logger(), "Theta variance (last 5): %f degree^2", variance*(180/M_PI)*(180/M_PI));
            // }
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr wall_pub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::deque<double> theta_values_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WallDetection>());
    rclcpp::shutdown();
    return 0;
}