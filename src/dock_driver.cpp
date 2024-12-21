/*
 * @Author: Wei Sun 
 * @Date: 2024-12-13
 */
#include "rclcpp/rclcpp.hpp"
#include "auto_dock/dock_driver.hpp"
#include "tf2/utils.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/impl/convert.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#define sign(x) (x>0?+1:x<0?-1:0)
#define setState(x) {state_=x;}
#define setStateVel(x,v,w) {setState(x);setVel(v,w);}

DockDriver::DockDriver() :
        Node("dock_driver")
        ,state_(RobotState::IDLE)
        ,state_str_("IDLE")
        ,vx_(0.0), wz_(0.0)
        ,ROBOT_STATE_STR(8)
        ,RDP_VALID(false)
{
    cmd_publisher_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    ROBOT_STATE_STR[0] = "IDLE";
    ROBOT_STATE_STR[1] = "SCAN";
    ROBOT_STATE_STR[2] = "FIND_DOCK";
    ROBOT_STATE_STR[3] = "GET_PARALLEL";
    ROBOT_STATE_STR[4] = "POSITION_ALIGN";
    ROBOT_STATE_STR[5] = "ANGLE_ALIGN";
    ROBOT_STATE_STR[6] = "DOCKING";
    ROBOT_STATE_STR[7] = "DOCKED_IN";

    IfFirstTime = true;
}

DockDriver::~DockDriver(){;}


void DockDriver::update(geometry_msgs::msg::PoseStamped::SharedPtr pose, const robot_interfaces::msg::DockPoseStamped::ConstSharedPtr relative_dock_pose)
{
    // ecl::LegacyPose2D<double> pose_update;
    double yaw_update;
    computePoseUpdate(yaw_update, pose);
    // updateVelocity(yaw_update, relative_dock_pose, pose);
    RCLCPP_INFO(this->get_logger(), "cmd update, vx_=%.6f. wz_=%.6f. state_=%s", vx_, wz_, state_str_);
    publishCmd(vx_,wz_);
}

void DockDriver::updateVelocity(double& yaw_update, const robot_interfaces::msg::DockPoseStamped::ConstSharedPtr relative_dock_pose, const geometry_msgs::msg::PoseStamped& pose)
{
    // RobotState::State current_state, new_state;
    // double new_vx = 0.0;
    // double new_wz = 0.0;

    // // determine the current state based on relative_dock_pose and the previous state
    // current_state = new_state = state_;
    // switch((unsigned int)current_state) {
    //     case RobotState::IDLE:
    //         idle(new_state, new_vx, new_wz);
    //         break;
    //     case RobotState::SCAN:
    //         scan(new_state, new_vx, new_wz);
    //         break;
    //     case RobotState::FIND_DOCK:
    //         find_dock(new_state, new_vx, new_wz, pose);
    //         break;
    //     case RobotState::GET_PARALLEL:
    //         get_parallel(new_state, new_vx, new_wz, relative_dock_pose);
    //         break;
    //     case RobotState::POSITION_ALIGN:
    //         position_align(new_state, new_vx, new_wz);
    //         break;
    //     case RobotState::ANGLE_ALIGN:
    //         angle_align(new_state, new_vx, new_wz, relative_dock_pose);
    //         break;
    //     case RobotState::DOCKING:
    //         docking(new_state, new_vx, new_wz, relative_dock_pose);
    //         break;
    //     case RobotState::DOCKED_IN:
    //         dock_in(new_state, new_vx, new_wz);
    //         break;
    //     default:
    //         ROS_INFO("-------Wrong state------");
    //         break;
    // }

    // setStateVel(new_state, new_vx, new_wz);
    // state_str_ = ROBOT_STATE_STR[(unsigned int)new_state];
}

void DockDriver::publishCmd(const double &vx, const double &wz){
    // vx_ = vx;
    // wz_ = wz;
    // twist_.linear.x = vx;
    // twist_.linear.y = 0;
    // twist_.linear.z = 0;
    // twist_.angular.x = 0;
    // twist_.angular.y = 0;
    // twist_.angular.z = wz;
    // cmd_publisher_->publish(twist_);
}


//根据odometry数据计算pose的变化量
void DockDriver::computePoseUpdate(double& yaw_update, geometry_msgs::msg::PoseStamped::SharedPtr pose)
{
    if(IfFirstTime){
        IfFirstTime = false;

        yaw_update = 0;
        pose_priv_ = move(pose);
    }else{
        double yaw = tf2::getYaw(pose->pose.orientation);
        yaw_update = tf2::getYaw(pose_priv_->pose.orientation) - yaw;
        pose_priv_ = move(pose);
    }
}

void DockDriver::setVel(double v, double w)
{
    vx_ = sign(v) * std::max(std::abs(v), MIN_ABS_V);//MIN_ABS_V 0.01
    wz_ = sign(w) * std::max(std::abs(w), MIN_ABS_W);//MIN_ABS_W 0.1
}