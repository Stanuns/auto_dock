#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <Eigen/Dense>
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/transform_broadcaster.h"
#include "robot_interfaces/msg/dock_pose_stamped.hpp"
#include "auto_dock/dock_driver.hpp"

#ifndef SRC_LIDAR_ALIGN_HPP
#define SRC_LIDAR_ALIGN_HPP
using namespace std;
using namespace Eigen;
using namespace rclcpp;


struct LinePara{ //直线参数 ax+by+c=0
    double a;
    double b;
    double c;
};

#define DOCK_LENGTH 0.33
#define DOCK_ORIENT 1 //DOCK_ORIENT为1：表示lidar的逆时针扫描方向，先遇到DOCK的长度为30mm的突出物
#define DOCK_STRUCTURE_KEY1  0.037 //表示lidar的逆时针扫描方向，先遇到DOCK的长度为30mm的突出物,此处限制了dock的摆放方式。
#define DOCK_STRUCTURE_KEY2  0.103
#define DOCK_STRUCTURE_KEY3  0.163
#define DOCK_STRUCTURE_KEY4  0.235
#define SCAN_COUNT_MIN  20
#define DOCK_LENGTH_MIN 0.27

class LidarAlign : public rclcpp::Node{
public:
    LidarAlign();
    ~LidarAlign();

protected:
void scanProc(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan);
double computeEuclideanDistance(geometry_msgs::msg::PointStamped& x1, geometry_msgs::msg::PointStamped& x2);
void getLinePara(geometry_msgs::msg::PointStamped& x1, geometry_msgs::msg::PointStamped& x2, LinePara& lp);
void getCrossPoint(LinePara& para1, LinePara& para2, geometry_msgs::msg::PointStamped& cp);
void publishTransform();
void publishLoop(double transform_publish_period);

private:
    Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    // Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr relative_dock_pose_pub_;
    Publisher<robot_interfaces::msg::DockPoseStamped>::SharedPtr relative_dock_pose_pub_;
    int scan_count;
    double r_inf;
    geometry_msgs::msg::PointStamped xs, xe, cross_point; //xs,xe:lidar 的 ray casting打到的点的坐标
    LinePara para_simul, para_ray;
//    vector<geometry_msgs::PointStamped> point_simul , point_scan;
    MatrixXf point_scan, point_simul, point_scan_cut;
    geometry_msgs::msg::PointStamped dock_key1_start, dock_key4_end, dock_center;
    // geometry_msgs::msg::PoseStamped dock_center_pose;
    robot_interfaces::msg::DockPoseStamped dock_center_pose;

    std::vector<float> ranges_filtered; 
    std::shared_ptr<std::thread> transform_thread_;
    double transform_publish_period_;
    std::mutex laser_to_dock_mutex_;
    tf2::Transform laser_to_dock_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tfB_;
    rclcpp::Node::SharedPtr node_;
    double thick_dock;


};




#endif //SRC_LIDAR_ALIGN_HPP
