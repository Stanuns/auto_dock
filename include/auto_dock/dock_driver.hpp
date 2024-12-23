/*
 * @Author: Wei Sun 
 * @Date: 2024-12-13
 */
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "auto_dock/dock_state.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "robot_interfaces/msg/dock_pose_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/utils.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/impl/convert.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


#ifndef SRC_DOCK_DRIVE_HPP
#define SRC_DOCK_DRIVE_HPP

#define  MIN_ABS_V  0.0
#define  MIN_ABS_W 0.0

#define  Threshold_RELEVANCE 0.0002

//根据不同的机器人设定
#define NEXT_VX 0.142
#define NEXT_WZ 0.316

using namespace std;
using namespace rclcpp;

class DockDriver : public rclcpp::Node{
public:
    DockDriver();
    ~DockDriver();

    RobotState::State getState() const { return state_; }
    std::string getStateStr() const { return state_str_; }
    void update(nav_msgs::msg::Odometry::SharedPtr odom, const robot_interfaces::msg::DockPoseStamped::ConstSharedPtr relative_dock_pose);
    void publishCmd(const double &vx, const double &wz);



    static inline double sign(double n){
        return n < 0.0 ? -1.0 : 1.0;
    }

protected:
    void computePoseUpdate(double& yaw_update, nav_msgs::msg::Odometry::SharedPtr odom);
    void updateVelocity(double& yaw_update, const robot_interfaces::msg::DockPoseStamped::ConstSharedPtr relative_dock_pose);
    void idle(RobotState::State& state, double& vx, double& wz); //
    void scan(RobotState::State& state, double& vx, double& wz, const robot_interfaces::msg::DockPoseStamped::ConstSharedPtr relative_dock_pose, double& yaw_update);
    void find_dock(RobotState::State& state,double& vx, double& wz, const robot_interfaces::msg::DockPoseStamped::ConstSharedPtr relative_dock_pose);
    void get_parallel(RobotState::State& state,double& vx, double& wz, const robot_interfaces::msg::DockPoseStamped::ConstSharedPtr relative_dock_pose);
    void position_align(RobotState::State& state,double& vx, double& wz, const robot_interfaces::msg::DockPoseStamped::ConstSharedPtr relative_dock_pose);
    void angle_align(RobotState::State& state,double& vx, double& wz, const robot_interfaces::msg::DockPoseStamped::ConstSharedPtr relative_dock_pose);
    void docking(RobotState::State& state,double& vx, double& wz, const robot_interfaces::msg::DockPoseStamped::ConstSharedPtr relative_dock_pose);
    void docked_in(RobotState::State& state,double& vx, double& wz, const robot_interfaces::msg::DockPoseStamped::ConstSharedPtr relative_dock_pose);

private:
    Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_publisher_;
    // Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    int dock_pos_detector_;
    double rotated_; //角度
    double angle_parallel_;
    double angle_align_pos_x_;
    int to_docking_count_;
    int to_position_align_count_;

    RobotState::State state_;
    std::string state_str_;
    double vx_, wz_;
    geometry_msgs::msg::Twist twist_;
    nav_msgs::msg::Odometry::SharedPtr odom_priv_; //计算computePoseUpdate时用到

    void setVel(double v, double w);
    std::vector<std::string> ROBOT_STATE_STR;
    bool RDP_VALID;
    bool IfFirstTime;

};


#endif //SRC_DOCK_DRIVE_HPP
