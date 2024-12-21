/*
 * @Author: Wei Sun 
 * @Date: 2024-12-17 10:18:47 
 * @Last Modified by: Wei Sun
 * @Last Modified time: 2024-12-20 15:25:29
 */

#include "auto_dock/dock_driver.hpp"

    /****************
     * Idle
     * @brief  Entry of auto docking state machine
     *
     * Shared variable
     *  @dock_detecotr   表示dock的位置. 正代表robot在dock的右边
     *  @rotated_    表示robot转动了多少.
    ******************/
    void DockDriver::idle(RobotState::State& nstate, double& nvx, double& nwz) {
        dock_detector_ = 0;
        rotated_ = 0.0;
        nstate = RobotState::SCAN;
        nvx = 0;
        nwz = 0.3;
    }


    /****************
     * Scan
     * @brief
     *
     * Shared variable
     * @dock_detector_ 表示dock的位置. 正代表robot在dock的右边.
     * @rotated_    表示robot转动了多少.
    ******************/
    void DockDriver::scan(RobotState::State& nstate, double& nvx, double& nwz, const robot_interfaces::msg::DockPoseStamped::ConstSharedPtr relative_dock_pose, double& yaw_update) 
    {
        RobotState::State next_state;
        double next_vx;
        double next_wz;

        double rdp_rele = relative_dock_pose->relevance;
        if(rdp_rele < Threshold_RELEVANCE){
            RDP_VALID = true;
        }else{
            RDP_VALID = false;
        }

        rotated_ += yaw_update*180/M_PI ;

        if(RDP_VALID == false)
        {
            next_state = RobotState::SCAN;
            next_vx = 0.0;
            next_wz = 0.3;
        }
        // robot is located left side of dock
        else if(RDP_VALID == true)
        {
            next_state = RobotState::FIND_DOCK;
            next_vx = 0.0;
            next_wz = 0.0;
        }
        else if(rotated_ > 380) { // 转动超过一圈
            next_state = RobotState::SCAN;
            next_vx = 0.0;
            next_wz = 0.0;

            rotated_ = 0;
        }

        RCLCPP_INFO(this->get_logger(), "scan rotated_= %.6f", rotated_);
        nstate = next_state;
        nvx = next_vx;
        nwz = next_wz;
    }

    void DockDriver::find_dock(RobotState::State& nstate,double& nvx, double& nwz, const robot_interfaces::msg::DockPoseStamped::ConstSharedPtr relative_dock_pose)
    {
        RobotState::State next_state;
        double next_vx;
        double next_wz;
        double rdp_rele = relative_dock_pose->relevance;
        if(rdp_rele < Threshold_RELEVANCE){
            RDP_VALID = true;
        }else{
            RDP_VALID = false;
        }

        if(RDP_VALID == true)
        {
            next_state = RobotState::FIND_DOCK;
            next_vx = 0.0;
            next_wz = 0.0;
        }

        nstate = next_state;
        nvx = next_vx;
        nwz = next_wz;
    }

    void DockDriver::get_parallel(RobotState::State& nstate,double& nvx, double& nwz, const robot_interfaces::msg::DockPoseStamped::ConstSharedPtr relative_dock_pose)
    {
        RobotState::State next_state;
        double next_vx;
        double next_wz;
        double rdp_rele = relative_dock_pose->relevance;
        if(rdp_rele < Threshold_RELEVANCE){
            RDP_VALID = true;
        }else{
            RDP_VALID = false;
        }

        if(RDP_VALID == true)
        {
            next_state = RobotState::GET_PARALLEL;
            next_vx = 0.0;
            next_wz = 0.0;
        }

        nstate = next_state;
        nvx = next_vx;
        nwz = next_wz;
    }
    void DockDriver::position_align(RobotState::State& nstate,double& nvx, double& nwz, const robot_interfaces::msg::DockPoseStamped::ConstSharedPtr relative_dock_pose)
    {
        RobotState::State next_state;
        double next_vx;
        double next_wz;
        double rdp_rele = relative_dock_pose->relevance;
        if(rdp_rele < Threshold_RELEVANCE){
            RDP_VALID = true;
        }else{
            RDP_VALID = false;
        }

        if(RDP_VALID == true)
        {
            next_state = RobotState::POSITION_ALIGN;
            next_vx = 0.0;
            next_wz = 0.0;
        }

        nstate = next_state;
        nvx = next_vx;
        nwz = next_wz;
    }
    void DockDriver::angle_align(RobotState::State& nstate,double& nvx, double& nwz, const robot_interfaces::msg::DockPoseStamped::ConstSharedPtr relative_dock_pose)
    {
        RobotState::State next_state;
        double next_vx;
        double next_wz;
        double rdp_rele = relative_dock_pose->relevance;
        if(rdp_rele < Threshold_RELEVANCE){
            RDP_VALID = true;
        }else{
            RDP_VALID = false;
        }

        if(RDP_VALID == true)
        {
            next_state = RobotState::ANGLE_ALIGN;
            next_vx = 0.0;
            next_wz = 0.0;
        }

        nstate = next_state;
        nvx = next_vx;
        nwz = next_wz;
    }
    void DockDriver::docking(RobotState::State& nstate,double& nvx, double& nwz, const robot_interfaces::msg::DockPoseStamped::ConstSharedPtr relative_dock_pose)
    {
        RobotState::State next_state;
        double next_vx;
        double next_wz;
        double rdp_rele = relative_dock_pose->relevance;
        if(rdp_rele < Threshold_RELEVANCE){
            RDP_VALID = true;
        }else{
            RDP_VALID = false;
        }

        if(RDP_VALID == true)
        {
            next_state = RobotState::DOCKING;
            next_vx = 0.0;
            next_wz = 0.0;
        }

        nstate = next_state;
        nvx = next_vx;
        nwz = next_wz;
    }
    void DockDriver::docked_in(RobotState::State& nstate,double& nvx, double& nwz, const robot_interfaces::msg::DockPoseStamped::ConstSharedPtr relative_dock_pose)
    {
        RobotState::State next_state;
        double next_vx;
        double next_wz;
        double rdp_rele = relative_dock_pose->relevance;
        if(rdp_rele < Threshold_RELEVANCE){
            RDP_VALID = true;
        }else{
            RDP_VALID = false;
        }

        if(RDP_VALID == true)
        {
            next_state = RobotState::DOCKED_IN;
            next_vx = 0.0;
            next_wz = 0.0;
        }

        nstate = next_state;
        nvx = next_vx;
        nwz = next_wz;
    }

