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
#include "robot_interfaces/msg/wall_pose_stamped.hpp"
#include "robot_interfaces/msg/dock_infra_red.hpp"


#ifndef SRC_DOCK_DRIVE_HPP
#define SRC_DOCK_DRIVE_HPP

#define  MIN_ABS_V  0.0
#define  MIN_ABS_W 0.0

#define  THRESHOLD_RELEVANCE 0.00024 //wheeltec:0.0002  luxshare:0.00024*1.2, 0.00148

//根据不同的机器人设定
#define NEXT_VX 0.142
#define NEXT_WZ 0.316
//1:代表激光雷达0角度是朝机器人以内(向后)的方向，-1：代表激光雷达0角度是朝机器人以外(向前)的方向. wheeltec与luxsharerobot激光雷达安装朝向一样
#define LIDAR_INSTALL_ORIENTATION 1; 

using namespace std;
using namespace rclcpp;

class DockDriver : public rclcpp::Node{
public:
    DockDriver();
    ~DockDriver();

    RobotState::State getState() const { return state_; }
    std::string getStateStr() const { return state_str_; }
    void update(nav_msgs::msg::Odometry::SharedPtr odom, const robot_interfaces::msg::DockPoseStamped::ConstSharedPtr relative_dock_pose,
                const robot_interfaces::msg::WallPoseStamped::ConstSharedPtr relative_wall_pose);
    void update_ir(nav_msgs::msg::Odometry::SharedPtr odom, const robot_interfaces::msg::DockInfraRed::ConstSharedPtr ir);
    void publishCmd(const double &vx, const double &wz);



    static inline double sign(double n){
        return n < 0.0 ? -1.0 : 1.0;
    }

protected:
    void computePoseUpdate(double& yaw_update, double& linear_update, nav_msgs::msg::Odometry::SharedPtr odom);
    void updateVelocity(double& yaw_update, double& linear_update, const robot_interfaces::msg::DockPoseStamped::ConstSharedPtr relative_dock_pose,
                        const robot_interfaces::msg::WallPoseStamped::ConstSharedPtr relative_wall_pose);
    void idle(RobotState::State& state, double& vx, double& wz); //
    void scan(RobotState::State& state, double& vx, double& wz, const robot_interfaces::msg::WallPoseStamped::ConstSharedPtr relative_wall_pose, 
                double& yaw_update);
    
    void find_wall(RobotState::State& state, double& vx, double& wz, const robot_interfaces::msg::WallPoseStamped::ConstSharedPtr relative_wall_pose);
    void scan2(RobotState::State& state, double& vx, double& wz, const robot_interfaces::msg::DockPoseStamped::ConstSharedPtr relative_dock_pose, 
                double& yaw_update);
    
    void find_dock(RobotState::State& state, double& vx, double& wz, const robot_interfaces::msg::DockPoseStamped::ConstSharedPtr relative_dock_pose);
    void get_parallel(RobotState::State& state, double& vx, double& wz, const robot_interfaces::msg::DockPoseStamped::ConstSharedPtr relative_dock_pose,
                        const robot_interfaces::msg::WallPoseStamped::ConstSharedPtr relative_wall_pose);
    
    void move_align(RobotState::State& state, double& vx, double& wz, double& linear_update);

    //未被使用
    void position_align(RobotState::State& state, double& vx, double& wz, const robot_interfaces::msg::DockPoseStamped::ConstSharedPtr relative_dock_pose);
    void position_align_extension(RobotState::State& state, double& vx, double& wz, double& linear_update);


    void angle_align(RobotState::State& state, double& vx, double& wz, const robot_interfaces::msg::DockPoseStamped::ConstSharedPtr relative_dock_pose);
    void docking(RobotState::State& state, double& vx, double& wz, const robot_interfaces::msg::DockPoseStamped::ConstSharedPtr relative_dock_pose,
                    const robot_interfaces::msg::WallPoseStamped::ConstSharedPtr relative_wall_pose);

    //未被使用                
    void turn_around(RobotState::State& state, double& vx, double& wz, double& yaw_update);
    void last_dock(RobotState::State& state, double& vx, double& wz); //有红外传感器, 需增加该传感器信息

    void docked_in(RobotState::State& state, double& vx, double& wz, const robot_interfaces::msg::DockPoseStamped::ConstSharedPtr relative_dock_pose);

    //InfraRed对接
    void updateVelocity_ir(double& yaw_update, double& linear_update, const robot_interfaces::msg::DockInfraRed::ConstSharedPtr ir);
    void scan_ir(RobotState::State& state, double& vx, double& wz, const robot_interfaces::msg::DockInfraRed::ConstSharedPtr ir, double& yaw_update);
    void find_ir(RobotState::State& state, double& vx, double& wz, const robot_interfaces::msg::DockInfraRed::ConstSharedPtr ir);
    void get_ir(RobotState::State& state, double& vx, double& wz, const robot_interfaces::msg::DockInfraRed::ConstSharedPtr ir);
    void scan_to_align_ir(RobotState::State& state, double& vx, double& wz, const robot_interfaces::msg::DockInfraRed::ConstSharedPtr ir, double& yaw_update);
    void aligned_ir(RobotState::State& state, double& vx, double& wz, const robot_interfaces::msg::DockInfraRed::ConstSharedPtr ir, double& yaw_update);
    void docking_ir(RobotState::State& state, double& vx, double& wz, const robot_interfaces::msg::DockInfraRed::ConstSharedPtr ir);
    void docked_in_ir(RobotState::State& state, double& vx, double& wz);



private:
    Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_publisher_;
    // Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    int dock_pos_detector_;
    int dock_detector_;
    double rotated_; //角度
    double linear_; //线运动距离
    double angle_parallel_;
    double position_align_pos_x_;
    int to_docking_count_;
    int to_position_align_count_;
    int to_move_align;
    int docked_in_count_;
    int to_last_dock_count_;
    int count_pae_;
    int to_find_wall_;

    RobotState::State state_;
    std::string state_str_;
    double vx_, wz_;
    geometry_msgs::msg::Twist twist_;
    nav_msgs::msg::Odometry::SharedPtr odom_priv_; //计算computePoseUpdate时用到

    void setVel(double v, double w);
    std::vector<std::string> ROBOT_STATE_STR;
    bool DOCK_VALID;
    bool WALL_VALID;
    bool IfFirstTime;
    // double relative_dock_pose_y_;
    double x_laser_inDock_;
    double y_laser_inDock_;
    int wall_size_threshold_;

    int get_ir_left_;
    int get_ir_right_;
    int docking_count_;
    int to_docking_ir;
};


#endif //SRC_DOCK_DRIVE_HPP
