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
        position_align_pos_x_ = -0.00 * LIDAR_INSTALL_ORIENTATION; //luxshare: -0.15; wheeltec:0.00
        to_position_align_count_ = 0;
        to_docking_count_ = 0;
        docked_in_count_ = 0;
        to_last_dock_count_ = 0;
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

        if(rdp_rele < THRESHOLD_RELEVANCE){
            RDP_VALID = true;
        }else{
            RDP_VALID = false;
        }

        rotated_ += yaw_update;

        if(RDP_VALID == true && fabs(pos_yaw) < 10) //机器人正对着dock
        {
            next_state = RobotState::FIND_DOCK;
            next_vx = 0.0;
            next_wz = 0.0;

            double pos_y =  relative_dock_pose->pose.position.y;
            if(pos_y < -0.05){
                dock_pos_detector_ = -1*LIDAR_INSTALL_ORIENTATION;
            }else if(fabs(pos_y) <= 0.05){
                dock_pos_detector_ = 0;
            }else if(pos_y > 0.05){
                dock_pos_detector_ = 1*LIDAR_INSTALL_ORIENTATION;
            }

            rotated_ = 0;
        }
        else if(fabs(rotated_) > 360+20) { // 转动超过一圈
            next_state = RobotState::SCAN;
            next_vx = 0.0;
            next_wz = 0.0;

            rotated_ = 0;
        }else{
            next_state = RobotState::SCAN;
            next_vx = 0.0;
            next_wz = 0.21;
        }

        RCLCPP_INFO(this->get_logger(), "scan rotated_= %.6f", rotated_);
        nstate = next_state;
        nvx = next_vx;
        nwz = next_wz;
    }

    void DockDriver::find_dock(RobotState::State& nstate, double& nvx, double& nwz, const robot_interfaces::msg::DockPoseStamped::ConstSharedPtr relative_dock_pose)
    {
        RobotState::State next_state;
        double next_vx;
        double next_wz;
        double rdp_rele = relative_dock_pose->relevance;

        double pos_yaw = tf2::getYaw(relative_dock_pose->pose.orientation)*180/M_PI;//值区间[-180 180]

        if(rdp_rele < THRESHOLD_RELEVANCE){
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
            angle_parallel_ = -65*LIDAR_INSTALL_ORIENTATION; //-90 由于luxsharerobot雷达视场角的遮挡，故改为-65
        }else if(dock_pos_detector_ == 1){  //机器人在dock右边
            next_state = RobotState::GET_PARALLEL;
            next_vx = 0.0;
            next_wz = 0.0;
            angle_parallel_ = 65*LIDAR_INSTALL_ORIENTATION; //90 由于luxsharerobot雷达视场角的遮挡，故改为65
        }else{
            next_state = RobotState::SCAN;
            next_vx = 0.0;
            next_wz = 0.0;
        }

        nstate = next_state;
        nvx = next_vx;
        nwz = next_wz;
    }

    void DockDriver::get_parallel(RobotState::State& nstate, double& nvx, double& nwz, const robot_interfaces::msg::DockPoseStamped::ConstSharedPtr relative_dock_pose)
    {
        RobotState::State next_state;
        double next_vx;
        double next_wz;
        double rdp_rele = relative_dock_pose->relevance;

        double pos_yaw = tf2::getYaw(relative_dock_pose->pose.orientation)*180/M_PI;

        if(rdp_rele < THRESHOLD_RELEVANCE){
            RDP_VALID = true;
        }else{
            RDP_VALID = false;
        }

        if(RDP_VALID == true && fabs(pos_yaw - angle_parallel_) < 10)
        {
            if(to_position_align_count_ > 2){
                next_state = RobotState::POSITION_ALIGN;
                to_position_align_count_ = 0;

                //对于luxsharerobot，当机器人在dock左边时，需要position_align_pos_x_
                if(dock_pos_detector_ < 0){
                    position_align_pos_x_ = position_align_pos_x_ * 1.6;
                }
            }else{
                next_state = RobotState::GET_PARALLEL;
                to_position_align_count_++;
            }

            next_vx = 0.1;

            //对于wheeltec, luxshare机器人存在bug，在旋转之后，单独给一个线速度，会有一定旋转，需要给一个反向角速度。
            if(dock_pos_detector_ < 0){
                next_wz = -0.6*LIDAR_INSTALL_ORIENTATION;
            }else{
                next_wz = 0.6*LIDAR_INSTALL_ORIENTATION;
            }
            

        }else if(RDP_VALID == true && dock_pos_detector_ > 0){
            next_state = RobotState::GET_PARALLEL;
            next_vx = 0.0;
            next_wz = -0.2*LIDAR_INSTALL_ORIENTATION;
        }else if(RDP_VALID == true && dock_pos_detector_ < 0){
            next_state = RobotState::GET_PARALLEL;
            next_vx = 0.0;
            next_wz = 0.2*LIDAR_INSTALL_ORIENTATION;
        }
        else if(dock_pos_detector_ > 0){
            next_state = RobotState::GET_PARALLEL;
            next_vx = 0.0;
            next_wz = -0.2*LIDAR_INSTALL_ORIENTATION;
        }else if(dock_pos_detector_ < 0){
            next_state = RobotState::GET_PARALLEL;
            next_vx = 0.0;
            next_wz = 0.2*LIDAR_INSTALL_ORIENTATION;
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

    void DockDriver::position_align(RobotState::State& nstate, double& nvx, double& nwz, const robot_interfaces::msg::DockPoseStamped::ConstSharedPtr relative_dock_pose)
    {
        RobotState::State next_state;
        double next_vx;
        double next_wz;
        double pos_yaw = tf2::getYaw(relative_dock_pose->pose.orientation)*180/M_PI;

        RCLCPP_INFO(this->get_logger(), "position_align, pos_yaw=%.6f.", pos_yaw);

        double rdp_rele = relative_dock_pose->relevance;
        if(rdp_rele < THRESHOLD_RELEVANCE){
            RDP_VALID = true;
        }else{
            RDP_VALID = false;
        }

        double pos_x =  relative_dock_pose->pose.position.x;

        if(RDP_VALID == true && fabs(pos_x - position_align_pos_x_) < 0.05)
        {
            next_state = RobotState::ANGLE_ALIGN;
            next_vx = 0.0;
            next_wz = 0.0;
        }else if(RDP_VALID == true && pos_x - position_align_pos_x_ < -0.05){
            next_state = RobotState::POSITION_ALIGN;
            next_vx = 0.1*LIDAR_INSTALL_ORIENTATION; //0.12
            next_wz = 0.0;
        }else if(RDP_VALID == true && pos_x - position_align_pos_x_ > 0.05){
            next_state = RobotState::POSITION_ALIGN;
            next_vx = -0.1*LIDAR_INSTALL_ORIENTATION; //-0.12
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

    void DockDriver::angle_align(RobotState::State& nstate, double& nvx, double& nwz, const robot_interfaces::msg::DockPoseStamped::ConstSharedPtr relative_dock_pose)
    {
        RobotState::State next_state;
        double next_vx;
        double next_wz;
        double rdp_rele = relative_dock_pose->relevance;
        double pos_yaw = tf2::getYaw(relative_dock_pose->pose.orientation)*180/M_PI;
        if(rdp_rele < THRESHOLD_RELEVANCE){
            RDP_VALID = true;
        }else{
            RDP_VALID = false;
        }

        RCLCPP_INFO(this->get_logger(), "angle_align------>dock_pos_detector_=%d.", dock_pos_detector_);

        if(RDP_VALID == true && fabs(pos_yaw) < 8)
        {
            if(to_docking_count_ > 3){
                next_state = RobotState::DOCKING;
                to_docking_count_ = 0;
                angle_parallel_ = 0;
            }else{
                next_state = RobotState::ANGLE_ALIGN;
                to_docking_count_++;
            }
            

            next_vx = 0.1;

            //对于wheeltec机器人存在bug，在旋转之后，单独给一个线速度，会有一定旋转，需要给一个反向角速度。
            if(dock_pos_detector_>0){
                next_wz = -0.6;
            }else if(dock_pos_detector_<0){
                next_wz = 0.6;
            }else{
                next_wz = -0.6; //直接从SCAN->FIND_DOCK->ANGLE_ALIGN
            }
            
        }else if(RDP_VALID == true && dock_pos_detector_ < 0){
            next_state = RobotState::ANGLE_ALIGN;
            next_vx = 0.0;
            next_wz = -0.25;
        }else if(RDP_VALID == true && dock_pos_detector_ > 0){
            next_state = RobotState::ANGLE_ALIGN;
            next_vx = 0.0;
            next_wz = 0.25;
        }
        // else if(dock_pos_detector_ < 0){
        //     next_state = RobotState::ANGLE_ALIGN;
        //     next_vx = 0.0;
        //     next_wz = -0.18;

        // }else if(dock_pos_detector_ > 0){
        //     next_state = RobotState::ANGLE_ALIGN;
        //     next_vx = 0.0;
        //     next_wz = 0.18;
        // }
        else{
            next_state = RobotState::ANGLE_ALIGN;
            next_vx = 0.0;
            next_wz = 0.25;
        }

        nstate = next_state;
        nvx = next_vx;
        nwz = next_wz;
    }

    void DockDriver::docking(RobotState::State& nstate, double& nvx, double& nwz, const robot_interfaces::msg::DockPoseStamped::ConstSharedPtr relative_dock_pose)
    {
        RobotState::State next_state;
        double next_vx;
        double next_wz;
        double pos_x =  relative_dock_pose->pose.position.x;
        double pos_y =  relative_dock_pose->pose.position.y;
        double pos_yaw = tf2::getYaw(relative_dock_pose->pose.orientation)*180/M_PI;
        RCLCPP_INFO(this->get_logger(), "docking, pos_yaw=%.6f.", pos_yaw);
        double rdp_rele = relative_dock_pose->relevance;
        if(rdp_rele < THRESHOLD_RELEVANCE){
            RDP_VALID = true;
        }else{
            RDP_VALID = false;
        }

        if(RDP_VALID == true && pos_x > -0.4)
        {
            next_state = RobotState::TURN_AROUND; //尾部对接需要，如头部对接，直接转到DOCKED_IN
            next_vx = 0.0;
            next_wz = 0.0;

            rotated_ = 0;
        }else if(RDP_VALID == true && pos_x < -0.4 && fabs(pos_y) < 0.1){
            next_state = RobotState::DOCKING;
            next_vx = 0.1;
            next_wz = 0.0;
        }else if(RDP_VALID == true && pos_x < -0.4 && pos_y < -0.1){
            next_state = RobotState::DOCKING;
            next_vx = 0.1;
            next_wz = 0.3;
        }else if(RDP_VALID == true && pos_x < -0.4 && pos_y > 0.1){
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
    
    //尾部对接需要该步骤
    void DockDriver::turn_around(RobotState::State& nstate, double& nvx, double& nwz, double& yaw_update)
    {
        RobotState::State next_state;
        double next_vx;
        double next_wz;

        rotated_ += yaw_update;

        if(rotated_ >= 175) //转动一圈
        {

            if(to_last_dock_count_ > 4){
                next_state = RobotState::LAST_DOCK;
                to_last_dock_count_ = 0;
                rotated_ = 0;
            }else{
                next_state = RobotState::TURN_AROUND;
                to_last_dock_count_++;
            }

            next_vx = -0.1;
            next_wz = -0.6;
        }
        else{
            next_state = RobotState::TURN_AROUND;
            next_vx = 0.0;
            next_wz = 0.25;
        }

        nstate = next_state;
        nvx = next_vx;
        nwz = next_wz;
    }

    //尾部对接需要该步骤，利用红外, 暂时硬件不支持
    void DockDriver::last_dock(RobotState::State& nstate, double& nvx, double& nwz)
    {
        RobotState::State next_state;
        double next_vx;
        double next_wz;

        if(docked_in_count_ > 10) 
        {
            next_state = RobotState::DOCKED_IN;
            next_vx = 0.0;
            next_wz = 0.0;

            docked_in_count_ = 0;
        }
        else{
            next_state = RobotState::LAST_DOCK;
            next_vx = -0.1;
            next_wz = 0.0;

            docked_in_count_++;
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
        if(rdp_rele < THRESHOLD_RELEVANCE){
            RDP_VALID = true;
        }else{
            RDP_VALID = false;
        }

        if(RDP_VALID == true) //头部进入充电桩
        {
            next_state = RobotState::DOCKED_IN;
            next_vx = 0.0;
            next_wz = 0.0;
        }else{  //尾部进入充电桩
            next_state = RobotState::DOCKED_IN;
            next_vx = 0.0;
            next_wz = 0.0;
        }

        nstate = next_state;
        nvx = next_vx;
        nwz = next_wz;
    }

