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
     *  @dock_pos_detector_   表示dock的位置. -1代表robot在dock的左边; 0代表robot在dock的中间; 1代表robot在dock的右边
     *  @rotated_    表示robot转动了多少.
    ******************/
    void DockDriver::idle(RobotState::State& nstate, double& nvx, double& nwz) {
        dock_pos_detector_ = -2;
        angle_parallel_ = 0;
        angle_align_pos_x_ = 0.23;
        to_position_align_count_ = 0;
        to_docking_count_ = 0;
        rotated_ = 0.0;
        nstate = RobotState::SCAN;
        nvx = 0;
        nwz = NEXT_WZ;
    }


    /****************
     * Scan
     * @brief
     *
     * Shared variable
     * @dock_pos_detector_ 表示dock的位置. -1代表robot在dock的左边; 0代表robot在dock的中间; 1代表robot在dock的右边
     * @rotated_    表示robot转动了多少.
    ******************/
    void DockDriver::scan(RobotState::State& nstate, double& nvx, double& nwz, const robot_interfaces::msg::DockPoseStamped::ConstSharedPtr relative_dock_pose, double& yaw_update) 
    {
        RobotState::State next_state;
        double next_vx;
        double next_wz;

        double rdp_rele = relative_dock_pose->relevance;
        double pos_yaw = tf2::getYaw(relative_dock_pose->pose.orientation)*180/M_PI;

        if(rdp_rele < Threshold_RELEVANCE){
            RDP_VALID = true;
        }else{
            RDP_VALID = false;
        }

        rotated_ += yaw_update;

        if(RDP_VALID == false)
        {
            next_state = RobotState::SCAN;
            next_vx = 0.0;
            next_wz = NEXT_WZ;
        }
        // robot is located left side of dock
        else if(RDP_VALID == true && fabs(pos_yaw) < 5) //机器人正对着dock
        {
            next_state = RobotState::FIND_DOCK;
            next_vx = 0.0;
            next_wz = 0.0;

            double pos_y =  relative_dock_pose->pose.position.y;
            if(pos_y < -0.08){
                dock_pos_detector_ = -1;
            }else if(fabs(pos_y) <= 0.08){
                dock_pos_detector_ = 0;
            }else if(pos_y > 0.08){
                dock_pos_detector_ = 1;
            }
        }
        else if(fabs(rotated_) > 360+20) { // 转动超过一圈
            next_state = RobotState::SCAN;
            next_vx = 0.0;
            next_wz = 0.0;

            rotated_ = 0;
        }else{
            next_state = RobotState::SCAN;
            next_vx = 0.0;
            next_wz = NEXT_WZ;
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

        double pos_yaw = tf2::getYaw(relative_dock_pose->pose.orientation)*180/M_PI;//值区间[-180 180]

        if(rdp_rele < Threshold_RELEVANCE){
            RDP_VALID = true;
        }else{
            RDP_VALID = false;
        }

        if(RDP_VALID == true && dock_pos_detector_ == 0){
                next_state = RobotState::ANGLE_ALIGN;
                next_vx = 0.0;
                next_wz = 0.0;
        }else if(dock_pos_detector_ == -1){ //机器人在dock左边
            next_state = RobotState::GET_PARALLEL;
            next_vx = 0.0;
            next_wz = 0.0;
            angle_parallel_ = -90;
        }else if(dock_pos_detector_ == 1){  //机器人在dock右边
            next_state = RobotState::GET_PARALLEL;
            next_vx = 0.0;
            next_wz = 0.0;
            angle_parallel_ = 90;
        }else{
            next_state = RobotState::SCAN;
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

        double pos_yaw = tf2::getYaw(relative_dock_pose->pose.orientation)*180/M_PI;

        if(rdp_rele < Threshold_RELEVANCE){
            RDP_VALID = true;
        }else{
            RDP_VALID = false;
        }

        if(RDP_VALID == true && fabs(pos_yaw - angle_parallel_) < 10)
        {
            if(to_position_align_count_ > 2){
                next_state = RobotState::POSITION_ALIGN;
                to_position_align_count_ = 0;
            }else{
                next_state = RobotState::GET_PARALLEL;
                to_position_align_count_++;
            }

            next_vx = 0.1;

            //对于wheeltec机器人存在bug，在旋转之后，单独给一个线速度，会有一定旋转，需要给一个反向角速度。
            if(angle_parallel_>0){
                next_wz = 0.6;
            }else{
                next_wz = -0.6;
            }
            

        }else if(RDP_VALID == true && angle_parallel_ > 0){
            next_state = RobotState::GET_PARALLEL;
            next_vx = 0.0;
            next_wz = -0.2;
        }else if(RDP_VALID == true && angle_parallel_ < 0){
            next_state = RobotState::GET_PARALLEL;
            next_vx = 0.0;
            next_wz = 0.2;
        }
        else if(angle_parallel_ > 0){
            next_state = RobotState::GET_PARALLEL;
            next_vx = 0.0;
            next_wz = -0.2;
        }else if(angle_parallel_ < 0){
            next_state = RobotState::GET_PARALLEL;
            next_vx = 0.0;
            next_wz = 0.2;
        }
        else{
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
        double pos_yaw = tf2::getYaw(relative_dock_pose->pose.orientation)*180/M_PI;

        RCLCPP_INFO(this->get_logger(), "position_align, pos_yaw=%.6f.", pos_yaw);

        double rdp_rele = relative_dock_pose->relevance;
        if(rdp_rele < Threshold_RELEVANCE){
            RDP_VALID = true;
        }else{
            RDP_VALID = false;
        }

        double pos_x =  relative_dock_pose->pose.position.x;

        if(RDP_VALID == true && fabs(pos_x - angle_align_pos_x_) < 0.05)
        {
            next_state = RobotState::ANGLE_ALIGN;
            next_vx = 0.0;
            next_wz = 0.0;
        }else if(RDP_VALID == true && pos_x - angle_align_pos_x_ < -0.05){
            next_state = RobotState::POSITION_ALIGN;
            next_vx = 0.1; //0.12
            next_wz = 0.0;
        }else if(RDP_VALID == true && pos_x - angle_align_pos_x_ > 0.05){
            next_state = RobotState::POSITION_ALIGN;
            next_vx = -0.1; //-0.12
            next_wz = 0.0;
        }else{
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
        double pos_yaw = tf2::getYaw(relative_dock_pose->pose.orientation)*180/M_PI;
        if(rdp_rele < Threshold_RELEVANCE){
            RDP_VALID = true;
        }else{
            RDP_VALID = false;
        }

        if(RDP_VALID == true && fabs(pos_yaw) < 8)
        {
            if(to_docking_count_ > 2){
                next_state = RobotState::DOCKING;
                to_docking_count_ = 0;
            }else{
                next_state = RobotState::ANGLE_ALIGN;
                to_docking_count_++;
            }
            

            next_vx = 0.1;

            //对于wheeltec机器人存在bug，在旋转之后，单独给一个线速度，回有一定旋转，需要给一个反向角速度。
            if(angle_parallel_>0){
                next_wz = -0.6;
            }else{
                next_wz = 0.6;
            }



            angle_parallel_ = 0;
        }else if(RDP_VALID == true && angle_parallel_ < 0){
            next_state = RobotState::ANGLE_ALIGN;
            next_vx = 0.0;
            next_wz = -0.25;
        }else if(RDP_VALID == true && angle_parallel_ > 0){
            next_state = RobotState::ANGLE_ALIGN;
            next_vx = 0.0;
            next_wz = 0.25;
        }
        else if(angle_parallel_ < 0){
            next_state = RobotState::ANGLE_ALIGN;
            next_vx = 0.0;
            next_wz = -0.2;

        }else if(angle_parallel_ > 0){
            next_state = RobotState::ANGLE_ALIGN;
            next_vx = 0.0;
            next_wz = 0.2;
        }
        else{
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
        double pos_x =  relative_dock_pose->pose.position.x;
        double pos_y =  relative_dock_pose->pose.position.y;
        double pos_yaw = tf2::getYaw(relative_dock_pose->pose.orientation)*180/M_PI;
        RCLCPP_INFO(this->get_logger(), "docking, pos_yaw=%.6f.", pos_yaw);
        double rdp_rele = relative_dock_pose->relevance;
        if(rdp_rele < Threshold_RELEVANCE){
            RDP_VALID = true;
        }else{
            RDP_VALID = false;
        }

        if(RDP_VALID == true && pos_x > -0.25)
        {
            next_state = RobotState::DOCKED_IN;
            next_vx = 0.0;
            next_wz = 0.0;
        }else if(RDP_VALID == true && pos_x < -0.25 && fabs(pos_y) < 0.1){
            next_state = RobotState::DOCKING;
            next_vx = 0.1;
            next_wz = 0.0;
        }else if(RDP_VALID == true && pos_x < -0.25 && pos_y < -0.1){
            next_state = RobotState::DOCKING;
            next_vx = 0.1;
            next_wz = 0.3;
        }else if(RDP_VALID == true && pos_x < -0.25 && pos_y > 0.1){
            next_state = RobotState::DOCKING;
            next_vx = 0.1;
            next_wz = -0.3;
        }else{
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
        }else{
            next_state = RobotState::DOCKED_IN;
            next_vx = 0.0;
            next_wz = 0.0;
        }

        nstate = next_state;
        nvx = next_vx;
        nwz = next_wz;
    }

