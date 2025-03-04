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
     *  @dock_detecotr_   表示dock的位置. 正代表robot在dock的右边
    ******************/
    void DockDriver::idle(RobotState::State& nstate, double& nvx, double& nwz) {
        dock_pos_detector_ = -2;
        angle_parallel_ = 0;
        position_align_pos_x_ = -0.07; //luxshare: -0.20; wheeltec:0.00
        to_position_align_count_ = 0;
        to_move_align = 0;
        to_docking_count_ = 0;
        docked_in_count_ = 0;
        to_last_dock_count_ = 0;
        count_pae_ = 0;
        rotated_ = 0.0;
        linear_ = 0.0;
        // nstate = RobotState::SCAN;
        nstate = RobotState::SCAN_IR;
        nvx = 0;
        nwz = NEXT_WZ;
        to_find_wall_ = 0;
        wall_size_threshold_ = 40;

        dock_detector_ = 0;  
        get_ir_left_ = 0;
        get_ir_right_  = 0;
        docking_count_ = 0;
    }


    /****************
     * scan
     * @brief
     * 墙壁检测
     * Shared variable
     * @rotated_    表示robot转动了多少.
    ******************/
    void DockDriver::scan(RobotState::State& nstate, double& nvx, double& nwz, const robot_interfaces::msg::WallPoseStamped::ConstSharedPtr relative_wall_pose, 
                            double& yaw_update) 
    {
        RobotState::State next_state;
        double next_vx;
        double next_wz;

        double wall_size = relative_wall_pose->size;
        double wall_pos_yaw = tf2::getYaw(relative_wall_pose->pose.orientation);

        if(wall_size > wall_size_threshold_){
            WALL_VALID = true;
        }else{
            WALL_VALID = false;
        }

        rotated_ += yaw_update;

        RCLCPP_INFO(this->get_logger(), "------------------------------scan wall_pos_yaw= %.6f", wall_pos_yaw*180/M_PI);

        if(WALL_VALID == true && ( fabs(wall_pos_yaw*180/M_PI) < 5 || fabs(wall_pos_yaw*180/M_PI - 180) < 5 || fabs(wall_pos_yaw*180/M_PI - (-180)) < 5 )) //机器人正对着wall
        {
            next_state = RobotState::FIND_WALL;
            
            if(to_find_wall_ > 3){
                next_state = RobotState::FIND_WALL;
                to_find_wall_ = 0;
                rotated_ = 0;
            }else{
                next_state = RobotState::SCAN;
                to_find_wall_++;
            }
            next_vx = 0.0;
            next_wz = -0.3;

            rotated_ = 0;
        }else if(fabs(rotated_*180/M_PI) > 360+20) { // 转动超过一圈
            next_state = RobotState::SCAN;
            next_vx = 0.0;
            next_wz = 0.0;

            rotated_ = 0;
            to_find_wall_ = 0;
        }else{
            next_state = RobotState::SCAN;
            next_vx = 0.0;
            next_wz = 0.21;
            to_find_wall_ = 0;
        }

        RCLCPP_INFO(this->get_logger(), "scan rotated_= %.6f", rotated_*180/M_PI);
        nstate = next_state;
        nvx = next_vx;
        nwz = next_wz;
    }

    void DockDriver::find_wall(RobotState::State& nstate, double& nvx, double& nwz, const robot_interfaces::msg::WallPoseStamped::ConstSharedPtr relative_wall_pose)
    {
        RobotState::State next_state;
        double next_vx;
        double next_wz;

        double wall_size = relative_wall_pose->size;

        if(wall_size > wall_size_threshold_){
            WALL_VALID = true;
        }else{
            WALL_VALID = false;
        }
        double dis_wall = relative_wall_pose->pose.position.x;

        if(WALL_VALID == true && fabs(dis_wall) < 0.5 ){
            next_state = RobotState::FIND_WALL;
            next_vx = -0.1;
            next_wz = 0.0;
        }else if(WALL_VALID == true && fabs(dis_wall) >= 0.5){
            next_state = RobotState::SCAN2;
            next_vx = 0.0;
            next_wz = 0.0;
        }else{
            next_state = RobotState::FIND_WALL;
            next_vx = 0.0;
            next_wz = 0.0;
        }

        RCLCPP_INFO(this->get_logger(), "find_wall dis_wall= %.6f", relative_wall_pose->pose.position.x);
        nstate = next_state;
        nvx = next_vx;
        nwz = next_wz;
    }

    /****************
     * scan2
     * @brief
     * dock检测
     * Shared variable
     * @dock_pos_detector_ 表示dock的位置. -1代表robot在dock的左边; 0代表robot在dock的中间; 1代表robot在dock的右边
     * @rotated_    表示robot转动了多少.
    ******************/
   void DockDriver::scan2(RobotState::State& nstate, double& nvx, double& nwz, const robot_interfaces::msg::DockPoseStamped::ConstSharedPtr relative_dock_pose, 
                            double& yaw_update) 
   {
       RobotState::State next_state;
       double next_vx;
       double next_wz;

       double rdp_rele = relative_dock_pose->relevance;
       double dock_pos_yaw = tf2::getYaw(relative_dock_pose->pose.orientation);

       if(rdp_rele < THRESHOLD_RELEVANCE){
           DOCK_VALID = true;
       }else{
           DOCK_VALID = false;
       }

       rotated_ += yaw_update;

       if(DOCK_VALID == true) //[-90,90]
       {
           //计算激光雷达到DOCK的垂直平分线的距离
           double k = tan(dock_pos_yaw);
           double C = relative_dock_pose->pose.position.y - k * relative_dock_pose->pose.position.x;
           double dis_laserToDock = fabs(C)/sqrt(k*k + 1);

           //坐标系变换,将在laser坐标系中的原点(laser)变换到dock坐标系中
           x_laser_inDock_ = -relative_dock_pose->pose.position.x*cos(dock_pos_yaw) - relative_dock_pose->pose.position.y*sin(dock_pos_yaw);
           y_laser_inDock_ = relative_dock_pose->pose.position.x*sin(dock_pos_yaw) - relative_dock_pose->pose.position.y*cos(dock_pos_yaw);
           
           RCLCPP_INFO(this->get_logger(), "------scan2: dis_laserToDock= %.6f, y_laser_inDock_ = %.6f", dis_laserToDock, y_laser_inDock_);

           next_state = RobotState::FIND_DOCK;
           next_vx = 0.0;
           next_wz = 0.0;

           if(y_laser_inDock_ < -0.05){
               dock_pos_detector_ = 1;
           }else if(fabs(y_laser_inDock_) <= 0.05){
               dock_pos_detector_ = 0;
           }else if(y_laser_inDock_ > 0.05){
               dock_pos_detector_ = -1;
           }

           rotated_ = 0;
       }else if(fabs(rotated_*180/M_PI) > 360+20) { // 转动超过一圈
           next_state = RobotState::SCAN2;
           next_vx = 0.0;
           next_wz = 0.0;

           rotated_ = 0;
       }else{
           next_state = RobotState::SCAN2;
           next_vx = 0.0;
           next_wz = 0.2;
       }

       RCLCPP_INFO(this->get_logger(), "scan2 rotated_= %.6f", rotated_*180/M_PI);
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

        double dock_pos_yaw = tf2::getYaw(relative_dock_pose->pose.orientation);//值区间[-180 180]

        if(rdp_rele < THRESHOLD_RELEVANCE){
            DOCK_VALID = true;
        }else{
            DOCK_VALID = false;
        }

        if(DOCK_VALID == true && dock_pos_detector_ == 0){
            next_state = RobotState::ANGLE_ALIGN;
            next_vx = 0.0;
            next_wz = 0.0;
        }else if(dock_pos_detector_ == -1){ //机器人在dock左边
            next_state = RobotState::GET_PARALLEL;
            next_vx = 0.0;
            next_wz = 0.0;
            // angle_parallel_ = -90; //-90 由于luxsharerobot雷达视场角的遮挡，故改为-65
        }else if(dock_pos_detector_ == 1){  //机器人在dock右边
            next_state = RobotState::GET_PARALLEL;
            next_vx = 0.0;
            next_wz = 0.0;
            // angle_parallel_ = 90; //90 由于luxsharerobot雷达视场角的遮挡，故改为65
        }else{
            next_state = RobotState::SCAN2;
            next_vx = 0.0;
            next_wz = 0.0;
        }

        nstate = next_state;
        nvx = next_vx;
        nwz = next_wz;
    }

    void DockDriver::get_parallel(RobotState::State& nstate, double& nvx, double& nwz, const robot_interfaces::msg::DockPoseStamped::ConstSharedPtr relative_dock_pose,
                                    const robot_interfaces::msg::WallPoseStamped::ConstSharedPtr relative_wall_pose)
    {
        RobotState::State next_state;
        double next_vx;
        double next_wz;

        double rdp_rele = relative_dock_pose->relevance;
        if(rdp_rele < THRESHOLD_RELEVANCE){
            DOCK_VALID = true;
        }else{
            DOCK_VALID = false;
        }

        int wall_size = relative_wall_pose->size;
        if(wall_size > wall_size_threshold_){
            WALL_VALID = true;
        }else{
            WALL_VALID = false;
        }

        double dock_pos_yaw = tf2::getYaw(relative_dock_pose->pose.orientation);
        double wall_pos_yaw = tf2::getYaw(relative_wall_pose->pose.orientation);
        double wall_pos_y = relative_wall_pose->pose.position.y;


        RCLCPP_INFO(this->get_logger(), "-----get_parallel dock_pos_yaw= %.6f, wall_pos_yaw=%.6f, dock_pos_detector_=%d, wall_size=%d", 
                                    dock_pos_yaw*180/M_PI, wall_pos_yaw*180/M_PI, dock_pos_detector_, wall_size);

        // if(DOCK_VALID == true && fabs(wall_pos_yaw - 90) < 5)
        if(WALL_VALID == true && fabs(wall_pos_yaw*180/M_PI - 90) < 5 && ((dock_pos_detector_ > 0 && wall_pos_y < 0) || (dock_pos_detector_ < 0 && wall_pos_y > 0)))
        {
            if(to_move_align > 2){
                next_state = RobotState::MOVE_ALIGN;
                to_move_align = 0;
                linear_ = 0;
                // //对于luxsharerobot，当机器人在dock左边时，需要position_align_pos_x_*1.6
                // if(dock_pos_detector_ < 0){
                //     position_align_pos_x_ = position_align_pos_x_;
                // }
            }else{
                next_state = RobotState::GET_PARALLEL;
                to_move_align++;
            }

            next_vx = 0.0;

            //对于wheeltec, luxshare机器人存在bug，在旋转之后，单独给一个线速度，会有一定旋转，需要给一个反向角速度。
            if(dock_pos_detector_ < 0){
                next_wz = -0.8;
            }else if(dock_pos_detector_ > 0){
                next_wz = 0.8;
            }
            

        }else if(WALL_VALID == true && dock_pos_detector_ > 0){
            next_state = RobotState::GET_PARALLEL;
            next_vx = 0.0;
            next_wz = -0.2;
        }else if(WALL_VALID == true &&  dock_pos_detector_ < 0){
            next_state = RobotState::GET_PARALLEL;
            next_vx = 0.0;
            next_wz = 0.2;
        }else if(dock_pos_detector_ > 0){
            next_state = RobotState::GET_PARALLEL;
            next_vx = 0.0;
            next_wz = -0.2;
        }else if(dock_pos_detector_ < 0){
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

    void DockDriver::move_align(RobotState::State& nstate, double& nvx, double& nwz, double& linear_update)
    {
        RobotState::State next_state;
        double next_vx;
        double next_wz;

        RCLCPP_INFO(this->get_logger(), "---------------move_align y_laser_inDock_= %.6f", y_laser_inDock_);

        linear_ += linear_update;
        if(linear_ > 0.3*fabs(y_laser_inDock_)){
            next_state = RobotState::SCAN2;
            next_vx = 0.0;
            next_wz = 0.0;

            linear_ = 0;
        }else if(linear_ < 0.3*fabs(y_laser_inDock_)){
            next_state = RobotState::MOVE_ALIGN;
            next_vx = 0.1;
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
        double dock_pos_yaw = tf2::getYaw(relative_dock_pose->pose.orientation);

        double rdp_rele = relative_dock_pose->relevance;
        if(rdp_rele < THRESHOLD_RELEVANCE){
            DOCK_VALID = true;
        }else{
            DOCK_VALID = false;
        }

        double dock_pos_x =  relative_dock_pose->pose.position.x;

        RCLCPP_INFO(this->get_logger(), "position_align, dock_pos_x=%.6f, position_align_pos_x_=%.6f.", dock_pos_x, position_align_pos_x_);
        if(DOCK_VALID == true && fabs(dock_pos_x - position_align_pos_x_) <= 0.04)
        {
            next_state = RobotState::POSITION_ALIGN_EXTENSION;
            next_vx = 0.0;
            next_wz = 0.0;
        }else if(DOCK_VALID == true && dock_pos_x - position_align_pos_x_ < -0.04){
            next_state = RobotState::POSITION_ALIGN;
            next_vx = 0.1; //0.12
            next_wz = 0.0;
        }else if(DOCK_VALID == true && dock_pos_x - position_align_pos_x_ > 0.04){
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

    void DockDriver::position_align_extension(RobotState::State& nstate, double& nvx, double& nwz, double& linear_update)
    {
        //由于激光雷达安装方式会造成遮挡
        //激光雷达定位dock丢失之后，继续向前移动一段距离，以达到position align dock的目的
        RobotState::State next_state;
        double next_vx;
        double next_wz;
        
        if(count_pae_ > 25){
            next_state = RobotState::ANGLE_ALIGN;
            next_vx = 0.0;
            next_wz = 0.0;
            count_pae_ = 0;
        }else{
            next_state = RobotState::POSITION_ALIGN_EXTENSION;
            next_vx = 0.1;
            next_wz = 0.0;
            count_pae_++;
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
        double dock_pos_yaw = tf2::getYaw(relative_dock_pose->pose.orientation);
        if(rdp_rele < THRESHOLD_RELEVANCE){
            DOCK_VALID = true;
        }else{
            DOCK_VALID = false;
        }

        RCLCPP_INFO(this->get_logger(), "angle_align------>dock_pos_detector_=%d.", dock_pos_detector_);

        if(DOCK_VALID == true && (fabs(dock_pos_yaw*180/M_PI-0) < 5 || fabs(dock_pos_yaw*180/M_PI-180) < 5 || fabs(dock_pos_yaw*180/M_PI-(-180)) < 5))
        {
            if(to_docking_count_ >= 4){

                if(fabs(dock_pos_yaw*180/M_PI-0) < 5){
                    next_state = RobotState::DOCKING;
                }else if(fabs(dock_pos_yaw*180/M_PI-180) < 5 || fabs(dock_pos_yaw*180/M_PI-(-180)) < 5){
                    next_state = RobotState::LAST_DOCK;
                }
                
                to_docking_count_ = 0;
                // angle_parallel_ = 0;
            }else{
                next_state = RobotState::ANGLE_ALIGN;
                to_docking_count_++;
            }

            next_vx = 0.0;
            //对于wheeltec机器人存在bug，在旋转之后，单独给一个线速度，会有一定旋转，需要给一个反向角速度。
            if(dock_pos_detector_>0){
                next_wz = 0.8;
            }else if(dock_pos_detector_<0){
                next_wz = -0.8;
            }else{
                next_wz = -1.2; //直接从SCAN->FIND_DOCK->ANGLE_ALIGN
            }

        }
        else if(to_docking_count_ >= 1){//对于wheeltec机器人存在bug，在旋转之后，单独给一个线速度，会有一定旋转，需要给一个反向角速度。
            if(to_docking_count_>=4){
                next_state = RobotState::ANGLE_ALIGN;
            }
            next_state = RobotState::DOCKING;
            next_vx = 0.0;
            
            if(dock_pos_detector_>0){
                next_wz = 0.8;
            }else if(dock_pos_detector_<0){
                next_wz = -0.8;
            }else{
                next_wz = -1.2; //直接从SCAN->FIND_DOCK->ANGLE_ALIGN
            }
            to_docking_count_++;
        }
        else{
            next_state = RobotState::ANGLE_ALIGN;
            next_vx = 0.0;
            next_wz = 0.21;
            to_docking_count_ = 0;
        }

        nstate = next_state;
        nvx = next_vx;
        nwz = next_wz;
    }

    void DockDriver::docking(RobotState::State& nstate, double& nvx, double& nwz, const robot_interfaces::msg::DockPoseStamped::ConstSharedPtr relative_dock_pose, 
        const robot_interfaces::msg::WallPoseStamped::ConstSharedPtr relative_wall_pose)
    {
        RobotState::State next_state;
        double next_vx;
        double next_wz;
        
        double dock_pos_x =  relative_dock_pose->pose.position.x;
        double dock_pos_y =  relative_dock_pose->pose.position.y;
        double dock_pos_yaw = tf2::getYaw(relative_dock_pose->pose.orientation);
        double rdp_rele = relative_dock_pose->relevance;
        if(rdp_rele < THRESHOLD_RELEVANCE){
            DOCK_VALID = true;
        }else{
            DOCK_VALID = false;
        }

        double wall_pos_x =  relative_wall_pose->pose.position.x;
        double wall_pos_y =  relative_wall_pose->pose.position.y;
        double wall_pos_yaw = tf2::getYaw(relative_wall_pose->pose.orientation);
        int wall_size = relative_wall_pose->size;
        if(wall_size > wall_size_threshold_){
            WALL_VALID = true;
        }else{
            WALL_VALID = false;
        }

        RCLCPP_INFO(this->get_logger(), "----------docking, dock_pos_yaw=%.6f, wall_pos_yaw=%.6f, wall_pos_x=%.6f", dock_pos_yaw*180/M_PI, wall_pos_yaw*180/M_PI, wall_pos_x);



        if(WALL_VALID == true && wall_pos_x > -0.2)
        {
            next_state = RobotState::DOCKED_IN; //尾部对接需要，如头部对接，直接转到DOCKED_IN
            next_vx = 0.0;
            next_wz = 0.0;

            rotated_ = 0;
        }else if(WALL_VALID == true && wall_pos_x < -0.2){ //需要借助红外
            next_state = RobotState::DOCKING;
            next_vx = 0.1;
            next_wz = 0.0;
        }
        // else if(WALL_VALID == true && wall_pos_x < -0.5 && pos_y < -0.1){
        //     next_state = RobotState::DOCKING;
        //     next_vx = 0.1;
        //     next_wz = 0.3;
        // }else if(WALL_VALID == true && wall_pos_x < -0.5 && pos_y > 0.1){
        //     next_state = RobotState::DOCKING;
        //     next_vx = 0.1;
        //     next_wz = -0.3;
        // }
        else{
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

        if(rotated_*180/M_PI >= 183) //转动一圈
        {

            if(to_last_dock_count_ > 4){
                next_state = RobotState::LAST_DOCK;
                to_last_dock_count_ = 0;
                rotated_ = 0;
            }else{
                next_state = RobotState::TURN_AROUND;
                to_last_dock_count_++;
            }

            next_vx = 0.0;
            next_wz = -0.3;
        }
        else{
            next_state = RobotState::TURN_AROUND;
            next_vx = 0.0;
            next_wz = 0.2;
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
            DOCK_VALID = true;
        }else{
            DOCK_VALID = false;
        }

        if(DOCK_VALID == true) //头部进入充电桩
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

    
    //infra red dock
    void DockDriver::scan_ir(RobotState::State& nstate, double& nvx, double& nwz, const robot_interfaces::msg::DockInfraRed::ConstSharedPtr ir, 
        double& yaw_update){
        RobotState::State next_state;
        double next_vx;
        double next_wz;

        unsigned char left  = ir->rec_left;
        unsigned char midback   = ir->rec_midback;
        unsigned char right = ir->rec_right;

        rotated_ += yaw_update;

        if(midback == DockStationIRState::CENTER)
        {
            next_state = RobotState::ALIGNED_IR;
            next_vx = 0.0;
            next_wz = 0.0;
            dock_detector_ = 0;
            rotated_ = 0;
        }
        // robot is located left side of dock
        else if(midback == DockStationIRState::LEFT || right == DockStationIRState::LEFT){
            dock_detector_--;
            next_state = RobotState::SCAN_IR;
            next_vx = 0.0;
            next_wz = 0.2;
        }
        // robot is located right side of dock
        else if(midback == DockStationIRState::RIGHT || left == DockStationIRState::RIGHT){
            dock_detector_++;
            next_state = RobotState::SCAN_IR;
            next_vx = 0.0;
            next_wz = 0.2;
        }
        else if(fabs(rotated_*180/M_PI) > 360+20) //在转了一圈之后 RobotState切换到FIND_IR
        {
            next_state = RobotState::FIND_IR;
            next_vx = 0;
            next_wz = 0.2;
            rotated_ = 0;
        }
        else { // if mid sensor does not see anything, rotate fast
            next_state = RobotState::SCAN_IR;
            next_vx = 0.0;
            next_wz = 0.2;
        }

        RCLCPP_INFO(this->get_logger(), "--scan_ir --dock_detector_: %d", dock_detector_);
        nstate = next_state;
        nvx = next_vx;
        nwz = next_wz;

    }
    void DockDriver::find_ir(RobotState::State& nstate, double& nvx, double& nwz, const robot_interfaces::msg::DockInfraRed::ConstSharedPtr ir){
        RobotState::State next_state;
        double next_vx;
        double next_wz;

        unsigned char left  = ir->rec_left;
        unsigned char midback   = ir->rec_midback;
        unsigned char right = ir->rec_right;

        RCLCPP_INFO(this->get_logger(), "--find_ir --dock_detector_: %d", dock_detector_);
        if(dock_detector_ > 0) // robot is located in right side of dock
        {
            if(right == DockStationIRState::RIGHT) { //|| midback == DockStationIRState::RIGHT
                next_state = RobotState::GET_IR;
                next_vx = 0.0;
                next_wz = 0.0;
            }
            else {
                next_state = RobotState::FIND_IR;
                next_vx = 0.0;
                next_wz = 0.2;
            }
        }
        else if(dock_detector_ < 0 ) // robot is located in left side of dock
        {
            if(left == DockStationIRState::LEFT) // || midback == DockStationIRState::LEFT
            {
                next_state = RobotState::GET_IR;
                next_vx = 0.0;
                next_wz = 0.0;
            }
            else {
                next_state = RobotState::FIND_IR;
                next_vx = 0.0;
                next_wz = -0.2;
            }
        }
        else{//机器人在中间区域逻辑需要完善
            if(midback == DockStationIRState::CENTER)
            {
                next_state = RobotState::ALIGNED_IR;
                next_vx = 0.0;
                next_wz = 0.0;
                dock_detector_ = 0;
                rotated_ = 0;
            }
            else
            {
                next_state = RobotState::FIND_IR;
                next_vx = 0.0;
                next_wz = 0.2;
            }
        }

        nstate = next_state;
        nvx = next_vx;
        nwz = next_wz;
    }
    void DockDriver::get_ir(RobotState::State& nstate, double& nvx, double& nwz, const robot_interfaces::msg::DockInfraRed::ConstSharedPtr ir){
        RobotState::State next_state;
        double next_vx;
        double next_wz;

        unsigned char left  = ir->rec_left;
        unsigned char midback   = ir->rec_midback;
        unsigned char right = ir->rec_right;

        if(dock_detector_ > 0) { // robot is located in right side of dock
        //    if (left == DockStationIRState::LEFT) {
            if (right == DockStationIRState::CENTER){
                get_ir_right_ = 0;
                dock_detector_ = 0;
                rotated_ = 0;
                next_state = RobotState::SCAN_TO_ALIGN_IR;
                next_vx = 0;
                next_wz = 0.2;
            }
            else{
                // get_ir_right_++;
                // if(get_ir_right_ > 20){
                //     dock_detector_ = 0;
                //     rotated_ = 0;
                //     next_state = RobotState::GO_BACK;
                //     next_vx = -0.1;
                //     next_wz = 0;
                //     get_ir_right_ = 0;
                // }
                // else{
                next_state = RobotState::GET_IR;
                next_vx = -0.1;
                next_wz = 0.0;
                // }
            }
        }
        else if(dock_detector_ < 0) { // robot is located left side of dock
            if (left == DockStationIRState::CENTER){
                get_ir_left_ = 0;
                dock_detector_ = 0;
                rotated_ = 0;
                next_state = RobotState::SCAN_TO_ALIGN_IR;
                next_vx = 0;
                next_wz = -0.2;
            }
            else {
                // get_ir_left_++;
                // if(get_ir_left_ > 20){
                //     dock_detector_ = 0;
                //     rotated_ = 0;
                //     next_state = RobotState::GO_BACK;
                //     next_vx = -0.1;
                //     next_wz = 0;
                //     get_ir_left_ = 0;
                // }
                // else{
                next_state = RobotState::GET_IR;
                next_vx = -0.1;
                next_wz = 0.0;
                // }
            }
        }
        else{
            next_state = RobotState::SCAN_IR;
            next_vx = 0.0;
            next_wz = 0.0;
        }

        nstate = next_state;
        nvx = next_vx;
        nwz = next_wz;
    }
    void DockDriver::scan_to_align_ir(RobotState::State& nstate, double& nvx, double& nwz, const robot_interfaces::msg::DockInfraRed::ConstSharedPtr ir, 
        double& yaw_update){
        RobotState::State next_state;
        double next_vx;
        double next_wz;

        unsigned char left  = ir->rec_left;
        unsigned char midback   = ir->rec_midback;
        unsigned char right = ir->rec_right;

        rotated_ += yaw_update;
        if(midback == DockStationIRState::CENTER)
        {
            next_state = RobotState::ALIGNED_IR;
            next_vx = 0.0;
            next_wz = 0.0;
            dock_detector_ = 0;
            rotated_ = 0;
        }
        // else if(left == DockStationIRState::CENTER)
        // {
        //     next_state = RobotState::SCAN_IR;
        //     next_vx = 0.0;
        //     next_wz = 0.2;
        //     rotated_ = 0;
        // }
        else if(fabs(rotated_*180/M_PI) > 360+20)
        {
            next_state = RobotState::SCAN_IR;
            next_vx = 0.0;
            next_wz = 0.0;
            rotated_ = 0;
        }
        else {
            next_state = RobotState::SCAN_TO_ALIGN_IR;
            next_vx = 0.0;
            next_wz = -0.2;
        }

        nstate = next_state;
        nvx = next_vx;
        nwz = next_wz;
    }
    void DockDriver::aligned_ir(RobotState::State& nstate, double& nvx, double& nwz, const robot_interfaces::msg::DockInfraRed::ConstSharedPtr ir, 
        double& yaw_update){
        RobotState::State next_state;
        double next_vx;
        double next_wz;

        unsigned char left  = ir->rec_left;
        unsigned char midback   = ir->rec_midback;
        unsigned char right = ir->rec_right;
        if(midback  == DockStationIRState::CENTER)
        {
            next_state = RobotState::DOCKING_IR;
            next_vx = 0.0;
            next_wz = 0.0;
        }
        else if(midback == DockStationIRState::LEFT) {
            next_state = RobotState::SCAN_IR;
            next_vx = 0.0;
            next_wz = 0.0;
        }
        else if(midback == DockStationIRState::RIGHT){
            next_state = RobotState::SCAN_IR;
            next_vx = 0.0;
            next_wz = 0.0;
        }
        else {
            next_state = RobotState::ALIGNED_IR;
            next_vx = 0.0;
            next_wz = 0.0;
        }

        nstate = next_state;
        nvx = next_vx;
        nwz = next_wz;   
    }
    void DockDriver::docking_ir(RobotState::State& nstate, double& nvx, double& nwz, const robot_interfaces::msg::DockInfraRed::ConstSharedPtr ir){
        RobotState::State next_state;
        double next_vx;
        double next_wz;

        unsigned char left  = ir->rec_left;
        unsigned char midback   = ir->rec_midback;
        unsigned char right = ir->rec_right;

        if(docking_count_ > 40) 
        {
            next_state = RobotState::DOCKED_IN_IR;
            next_vx = 0.0;
            next_wz = 0.0;

            docking_count_ = 0;
        }
        else if(midback == DockStationIRState::CENTER){
            next_state = RobotState::DOCKING_IR;
            next_vx = -0.1;
            next_wz = 0.0;

            docking_count_++;
        }else{
            next_state = RobotState::DOCKING_IR;
            next_vx = 0.0;
            next_wz = 0.0;

            // docking_count_++;
        }

        nstate = next_state;
        nvx = next_vx;
        nwz = next_wz;
    }
    void DockDriver::docked_in_ir(RobotState::State& nstate, double& nvx, double& nwz){
        RobotState::State next_state;
        double next_vx;
        double next_wz;

        next_state = RobotState::DOCKED_IN_IR;
        next_vx = 0.0;
        next_wz = 0.0;

        nstate = next_state;
        nvx = next_vx;
        nwz = next_wz;
    }

