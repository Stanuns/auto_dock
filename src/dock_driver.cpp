/*
 * @Author: Wei Sun 
 * @Date: 2024-12-13
 */
#include "rclcpp/rclcpp.hpp"
#include "auto_dock/dock_driver.hpp"

#define sign(x) (x>0?+1:x<0?-1:0)
#define setState(x) {state_=x;}
#define setStateVel(x,v,w) {setState(x);setVel(v,w);}

DockDriver::DockDriver() :
        Node("dock_driver")
        ,state_(RobotState::IDLE)
        ,state_str_("IDLE")
        ,vx_(0.0), wz_(0.0)
        ,ROBOT_STATE_STR(11)
        ,RDP_VALID(false)
{
    cmd_publisher_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    ROBOT_STATE_STR[0] = "IDLE";
    ROBOT_STATE_STR[1] = "SCAN";
    ROBOT_STATE_STR[2] = "FIND_DOCK";
    ROBOT_STATE_STR[3] = "GET_PARALLEL";
    ROBOT_STATE_STR[4] = "POSITION_ALIGN";
    ROBOT_STATE_STR[5] = "POSITION_ALIGN_EXTENSION";
    ROBOT_STATE_STR[6] = "ANGLE_ALIGN";
    ROBOT_STATE_STR[7] = "DOCKING";
    ROBOT_STATE_STR[8] = "TURN_AROUND";
    ROBOT_STATE_STR[9] = "LAST_DOCK";
    ROBOT_STATE_STR[10] = "DOCKED_IN";

    IfFirstTime = true;
}

DockDriver::~DockDriver(){;}


void DockDriver::update(nav_msgs::msg::Odometry::SharedPtr odom, const robot_interfaces::msg::DockPoseStamped::ConstSharedPtr relative_dock_pose)
{
    // ecl::LegacyPose2D<double> pose_update;
    double yaw_update, linear_update;
    computePoseUpdate(yaw_update, linear_update, odom);
    updateVelocity(yaw_update, linear_update, relative_dock_pose);
    RCLCPP_INFO(this->get_logger(), "cmd update, vx_=%.6f. wz_=%.6f. state_=%s", vx_, wz_, state_str_.c_str());
    publishCmd(vx_,wz_);
}

void DockDriver::updateVelocity(double& yaw_update, double& linear_update, const robot_interfaces::msg::DockPoseStamped::ConstSharedPtr relative_dock_pose)
{
    RobotState::State current_state, new_state;
    double new_vx = 0.0;
    double new_wz = 0.0;

    // determine the current state based on relative_dock_pose and the previous state
    current_state = new_state = state_;
    switch((unsigned int)current_state) {
        case RobotState::IDLE:
            idle(new_state, new_vx, new_wz);
            break;
        case RobotState::SCAN:
            scan(new_state, new_vx, new_wz, relative_dock_pose, yaw_update);
            break;
        case RobotState::FIND_DOCK:
            find_dock(new_state, new_vx, new_wz, relative_dock_pose);
            break;
        case RobotState::GET_PARALLEL:
            get_parallel(new_state, new_vx, new_wz, relative_dock_pose);
            break;
        case RobotState::POSITION_ALIGN:
            position_align(new_state, new_vx, new_wz, relative_dock_pose);
            break;
        case RobotState::POSITION_ALIGN_EXTENSION:
            position_align_extension(new_state, new_vx, new_wz, linear_update);
            break;
        case RobotState::ANGLE_ALIGN:
            angle_align(new_state, new_vx, new_wz, relative_dock_pose);
            break;
        case RobotState::DOCKING:
            docking(new_state, new_vx, new_wz, relative_dock_pose);
            break;
        case RobotState::TURN_AROUND:
            turn_around(new_state, new_vx, new_wz, yaw_update);
            break;
        case RobotState::LAST_DOCK:
            last_dock(new_state, new_vx, new_wz);
            break;
        case RobotState::DOCKED_IN:
            docked_in(new_state, new_vx, new_wz, relative_dock_pose);
            break;
        default:
            RCLCPP_INFO(this->get_logger(), "-----wrong state-----");
            break;
    }

    setStateVel(new_state, new_vx, new_wz);
    state_str_ = ROBOT_STATE_STR[(unsigned int)new_state];
}

void DockDriver::publishCmd(const double &vx, const double &wz){
    vx_ = vx;
    wz_ = wz;
    twist_.linear.x = vx;
    twist_.linear.y = 0;
    twist_.linear.z = 0;
    twist_.angular.x = 0;
    twist_.angular.y = 0;
    twist_.angular.z = wz;
    cmd_publisher_->publish(twist_);
}


//根据odometry数据计算pose的变化量
void DockDriver::computePoseUpdate(double& yaw_update, double& linear_update, nav_msgs::msg::Odometry::SharedPtr odom)
{
    if(IfFirstTime){
        IfFirstTime = false;

        yaw_update = 0;
        linear_update = 0;
        odom_priv_ = move(odom);
    }else{
        double yaw_current = tf2::getYaw(odom->pose.pose.orientation); //值区间[-180 180]
        yaw_update = yaw_current - tf2::getYaw(odom_priv_->pose.pose.orientation);
        yaw_update = yaw_update*180/M_PI;
        //值区间映射到[-180,180]
        if(yaw_update < -180){ 
            yaw_update = yaw_update + 360;
        }else if(yaw_update > 180){
            yaw_update = yaw_update - 360;
        }

        double linear_current = odom->pose.pose.position;
        linear_update = sqrt(pow(linear_current.x - odom_priv_->pose.pose.position.x, 2) + pow(linear_current.y - odom_priv_->pose.pose.position.y, 2));


        odom_priv_ = move(odom);
    }
}

void DockDriver::setVel(double v, double w)
{
    vx_ = sign(v) * std::max(std::abs(v), MIN_ABS_V);//MIN_ABS_V 0.01
    wz_ = sign(w) * std::max(std::abs(w), MIN_ABS_W);//MIN_ABS_W 0.1
}