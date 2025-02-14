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

#define MAX_RANGES 1.5
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
            if (r > MAX_RANGES || isinf(r) || std::isnan(r)) {
                ranges_filtered[i] = R_INF;
            } else {
                ranges_filtered[i] = r;
            }
        }

        // Smooth laser ranges
        for (size_t i = 1; i < scan_count - 2; i++) {
            if (ranges_filtered[i] > R_INF-1  && (fabs(ranges_filtered[i + 1] - ranges_filtered[i - 1]) < 0.03)) {
                ranges_filtered[i] = (ranges_filtered[i + 1] + ranges_filtered[i - 1]) / 2.0f;
            } else if (ranges_filtered[i] > R_INF-1 && ranges_filtered[i + 1] > R_INF-1 &&
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
            // point.x = scan->ranges[i] * cos(angle);
            // point.y = scan->ranges[i] * sin(angle);
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

            wall_poses.poses.push_back(pose);
            wall_pub_->publish(wall_poses);
            RCLCPP_INFO(this->get_logger(), "Detected wall direction angle: %f radians", (theta*180)/M_PI);

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
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr wall_pub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WallDetection>());
    rclcpp::shutdown();
    return 0;
}