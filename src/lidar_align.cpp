/**
 * 每次进行autodocking时 进入该部分只取一次LaserScan的值
 *
 * 根据LaserScan的取值发布 dock的关键部位 在 Lidar坐标系（Lidar_coord）下的坐标
 * 该Lidar坐标满足以下要求：
 * 1）以Lidar当前位姿为坐标原点（0.0.0）（需要利用tf转换到机器人本体坐标系中的坐标）
 * 2) 以Lidar的0角度为x轴正方向 逆时针旋转pi/2角度为y轴正方向: 
 * wheeltec：Lidar 0角度是往自身正中心方向（以为），逆时针旋转为正方向
 * 3）随着rplidar的角度增大（0 ～ 2×pi），在坐标轴上的表现是一个逆时针的过程
 * 将该数据发布到topic    /relative_dock_pose
 * **/

#include <auto_dock/lidar_align.hpp>
#include <math.h>
#include "tf2/utils.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/impl/convert.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
using namespace Eigen;

LidarAlign::LidarAlign():
Node("lidar_align") {
    r_inf = 100.0; //定义当lidar扫描的值为std::numeric_limits<double>::infinity()时的一个较大值10^6，方便后续计算相关度
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>("/scan", 1, 
                std::bind(&LidarAlign::scanProc, this, std::placeholders::_1));
    relative_dock_pose_pub_= create_publisher<geometry_msgs::msg::PoseStamped>(//发送此数据到底盘进行控制
                        "/relative_dock_pose", 10);
    cross_point.point.z=0;
}

LidarAlign::~LidarAlign() { ; }


void LidarAlign::scanProc(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan) {

    scan_count = scan->ranges.size();
    double r, theta, rele_temp, dock_theta;
    geometry_msgs::msg::PointStamped x1_temp,x2_temp,xe_temp;
    geometry_msgs::msg::Quaternion dock_pose_quat;
    tf2::Quaternion tf2_quat;

    //debug
    RCLCPP_INFO(this->get_logger(), "scan: ranges[185]:%.6f, ranges[186]:%.6f, ranges[187]:%.6f, ranges[188]:%.6f", scan->ranges[185],scan->ranges[186],scan->ranges[187],scan->ranges[188]);
    RCLCPP_INFO(this->get_logger(), "scan: ranges[189]:%.6f, ranges[190]:%.6f, ranges[191]:%.6f, ranges[192]:%.6f", scan->ranges[189],scan->ranges[190],scan->ranges[191],scan->ranges[192]);
    
    //对scan进行过滤, 对于wheeltec的激光雷达有nan的值
    ranges_filtered.resize(scan_count);
    for(int i = 0; i < scan_count; i++){
        r = scan->ranges[i];
        if( r > scan->range_max || std::isnan(r) || std::isinf(r)){
            ranges_filtered[i] = r_inf;
        }else{
            ranges_filtered[i] = r;
        }
    }
    for(int i = 1; i < scan_count-2; i++){
        if(ranges_filtered[i] > r_inf-1 && (fabs(ranges_filtered[i+1] - ranges_filtered[i-1]) < 0.03)){
            ranges_filtered[i] = (ranges_filtered[i+1] + ranges_filtered[i-1]) / 2.0;
        }else if(ranges_filtered[i] > r_inf-1 && ranges_filtered[i+1] > r_inf-1 && (fabs(ranges_filtered[i+2] - ranges_filtered[i-1]) < 0.03)){
            ranges_filtered[i] = ranges_filtered[i+1] = (ranges_filtered[i+2] + ranges_filtered[i-1]) / 2.0;
        }

    }



    //存储各个 point_simul，point_scan 相关度的值
    point_scan = MatrixXf::Zero(2,scan_count);
    for(int i=0; i<scan_count; i++){
        // r = scan->ranges[i];
        // if(r > scan->range_max || std::isnan(r)){
        //     point_scan(0,i) = r_inf;
        //     point_scan(1,i) = r_inf;
        // }else{
        //     theta = scan->angle_min + scan->angle_increment * i + 0.0;
        //     point_scan(0,i) = r * cos(theta);
        //     point_scan(1,i) = r * sin(theta);
        // }
        theta = scan->angle_min + scan->angle_increment * i + 0.0;
        point_scan(0,i) = ranges_filtered[i] * cos(theta);
        point_scan(1,i) = ranges_filtered[i] * sin(theta);

    }
    //    VectorXf rele(scan_count);
//        VectorXf rele111 = VectorXf::Zero(scan_count);
    VectorXf rele = VectorXf::Constant(scan_count,std::numeric_limits<double>::infinity());
    // 存储每个rele对应的point_scan_cut的终点序号,注意：起点index就是i
    VectorXi index_end = VectorXi::Zero(scan_count);


    for (int i = 0; i < scan_count; i++) {
        // r = scan->ranges[i];
        r = ranges_filtered[i];
        theta = scan->angle_min + scan->angle_increment * i + 0.0;
        if(r > scan->range_max){
            continue;
        }
        xs.point.x = r * cos(theta);
        xs.point.y = r * sin(theta);
        xs.point.z = 0;

        //得到xe;
        int count_simul = 1; //计算一个DOCK_LENGTH长度的lidar打出的点数（range是inf时，不算长度，但是算作一个点数）
        int count_simul_temp = 1;
        for (int j = i + 1; j < (int)(scan_count*1.5); j++) {

            if(j >= scan_count){
                // r = scan->ranges[j-scan_count];
                r = ranges_filtered[j-scan_count];
                theta = scan->angle_min + scan->angle_increment * (j-scan_count) + 0.0;
            }else{
                // r = scan->ranges[j];
                r = ranges_filtered[j];
                theta = scan->angle_min + scan->angle_increment * j + 0.0;
            }

            if(r > scan->range_max){
                continue;
            }

            xe_temp.point.x = r * cos(theta);
            xe_temp.point.y = r * sin(theta);
            xe_temp.point.z = 0;
            count_simul_temp = j-i +1;
            double d = computeEuclideanDistance(xs, xe_temp);
            if (d >= DOCK_LENGTH) {
                break;
            }
            count_simul = count_simul_temp;
            xe = xe_temp;
        }
        if (count_simul < SCAN_COUNT_MIN){
            continue;
        }

        point_simul = MatrixXf::Zero(2,count_simul);
        point_simul(0,0) = xs.point.x;
        point_simul(1,0) = xs.point.y;

        //保存需要对比的point_scan的index
        VectorXi index_scan = VectorXi::Zero(count_simul);
        index_scan(0) = i;

        //求xs_xe的连线与 lidar ray casting 射线的交点
        //求xs xe的连线的a，b，c
        getLinePara(xs, xe, para_simul);
        //求lidar_ray的a，b，c, 并求出lidar_ray与xs_xe的交点
        geometry_msgs::msg::PointStamped ray_x1,ray_x2;
        ray_x1.point.x = 0;
        ray_x1.point.y = 0;
        ray_x1.point.z = 0;
        for (int k = i+1; k < i+count_simul; k++){
            if(k >= scan_count){
                // r = scan->ranges[k-scan_count];
                r = ranges_filtered[k-scan_count];
                theta = scan->angle_min + scan->angle_increment * (k-scan_count) + 0.0;
            }else{
                // r = scan->ranges[k];
                r = ranges_filtered[k];
                theta = scan->angle_min + scan->angle_increment * k + 0.0;
            }

            ray_x2.point.x = 10*cos(theta);
            ray_x2.point.y = 10*sin(theta);
            ray_x2.point.z = 0;

            getLinePara(ray_x1, ray_x2, para_ray);
            //得到交点cross_point
            getCrossPoint(para_simul, para_ray, cross_point);
            double d_temp = computeEuclideanDistance(xs,cross_point);
            if ((d_temp > DOCK_STRUCTURE_KEY1 && d_temp < DOCK_STRUCTURE_KEY2) || (d_temp > DOCK_STRUCTURE_KEY3 && d_temp < DOCK_STRUCTURE_KEY4))    //该处来源与DOCK的形状以及安装的朝向
            {
                // point_simul(0,k-i) = r_inf; //不需要进行 * cos(theta)
                // point_simul(1,k-i) = r_inf;
                point_simul(0,k-i) = cross_point.point.x - 0.037;  //0.037特征结构件厚度
                point_simul(1,k-i) = cross_point.point.y;
            }else{
                point_simul(0,k-i) = cross_point.point.x;
                point_simul(1,k-i) = cross_point.point.y;
            }

            if(k >= scan_count){
                index_scan(k-i) = k - scan_count;
            }else{
                index_scan(k-i) = k;
            }


        }

        point_scan_cut = MatrixXf::Zero(2,count_simul);
        for (int m=0; m<count_simul; m++){
            point_scan_cut.col(m) = point_scan.col(index_scan(m));
        }
        /*
         * 对point_scan_cut进行判断
         * 求point_scan_cut中两点相距的最大距离
         * **/
        double dd_max = 0;
        for (int n=1;n<count_simul;n++){
            double r_temp = sqrt(pow(point_scan_cut(0,n),2)+pow(point_scan_cut(1,n),2));
            if(r_temp > scan->range_max || std::isnan(r)){
                continue;
            }
            x1_temp.point.x = point_scan_cut(0,n);
            x1_temp.point.y = point_scan_cut(1,n);
            x1_temp.point.z = 0;
            x2_temp.point.x = point_scan_cut(0,0);
            x2_temp.point.y = point_scan_cut(1,0);
            x2_temp.point.z = 0;

            double dd_temp = computeEuclideanDistance(x1_temp, x2_temp);
            if(dd_temp > dd_max){
                dd_max = dd_temp;
            }
        }
        if(dd_max < DOCK_LENGTH_MIN){
            continue;
        }



        //计算point_simul与point_scan数据的相关度 --误差的平方和
        rele(i)= 0.0;
        for (int m=0; m<count_simul; m++){
            rele(i) = rele(i) + pow(point_simul(0,m) - point_scan_cut(0,m),2) + pow(point_simul(1,m) - point_scan_cut(1,m),2);
        }
        rele(i) = rele(i)/count_simul;
        index_end(i) = index_scan(count_simul-1);

        //test
        //point_scan_cut.cols() 与 point_simul.cols() 的值与 count_simul 的值一般都是相等的
        //point_scan_cut.cols()的值与point_scan.cols()的值是相等的
        if(point_simul.cols() != count_simul){
            RCLCPP_INFO(this->get_logger(),"--test--1--");
        }else if(point_simul.cols() != point_scan_cut.cols()){
            RCLCPP_INFO(this->get_logger(),"--test--2--");
        }
//        ROS_INFO_STREAM("i="<<i<<", rele:"<<rele(i));

    }


    //求VectorXf最小值所在的行数
    VectorXf::Index minRow,minCol;
    double min_value = rele.minCoeff(&minRow,&minCol);
    //求dock在lidar坐标系下的位置
    // r = scan->ranges[minRow];
    r = ranges_filtered[minRow];
    theta = scan->angle_min + scan->angle_increment * minRow + 0.0;
    dock_key1_start.point.x = r*cos(theta);
    dock_key1_start.point.y = r*sin(theta);
    dock_key1_start.point.z = 0;
    // r = scan->ranges[index_end(minRow)];
    r = ranges_filtered[index_end(minRow)];
    theta = scan->angle_min + scan->angle_increment * index_end(minRow) + 0.0;
    dock_key4_end.point.x = r*cos(theta);
    dock_key4_end.point.y = r*sin(theta);
    dock_key4_end.point.z = 0;

    /**
     * 将lidar坐标系下的坐标转换成机器人本体的坐标
     * //todo 目前没用到tf转换
     * 在X轴方向上，rplidar A3的安装位置在两轮中心点的+0.243处
     * **/
//    dock_key1_start.point.x = dock_key1_start.point.x + 0.243;
//    dock_key4_end.point.x = dock_key4_end.point.x + 0.243;

    //求dock_key1_start 与 dock_key4_end 连线的中点
    dock_center.point.x = (dock_key1_start.point.x+dock_key4_end.point.x) / 2;
    dock_center.point.y = (dock_key1_start.point.y+dock_key4_end.point.y) / 2;
    dock_center.point.z = 0;
    //射线xs-xe的方向逆时针旋转M_PI/2  xs是DOCK_STRUCTURE_KEY1端  xe是DOCK_STRUCTURE_KEY4端
    dock_theta =  atan2(dock_key4_end.point.y-dock_key1_start.point.y, dock_key4_end.point.x-dock_key1_start.point.x) + M_PI/2;
    tf2_quat.setRPY(0, 0, dock_theta);
    dock_pose_quat = tf2::toMsg(tf2_quat);//将偏航角转换成四元数

      //test
//    if(dock_center.point.x > 1000){
//        int aa = index_end(minRow);
//      ROS_INFO("--test--");
//    }

    dock_center_pose.header.stamp = this->get_clock()->now();
    dock_center_pose.header.frame_id = "laser";
    dock_center_pose.pose.position.x = dock_center.point.x ;
    dock_center_pose.pose.position.y = dock_center.point.y ;
    dock_center_pose.pose.position.z = dock_center.point.z ;
    dock_center_pose.pose.orientation = dock_pose_quat;

    //过滤：不发布dock_center.point.x = inf 或者 dock_center.point.y = inf 的
    if(dock_center.point.x < std::numeric_limits<double>::infinity()  && dock_center.point.y < std::numeric_limits<double>::infinity()){
        relative_dock_pose_pub_->publish(dock_center_pose);
    }else{
        //RCLCPP_INFO(this->get_logger(), "There is a inf value in:  %.6f or  %.6f ", dock_center.point.x, dock_center.point.y);
    }

//    ROS_INFO_STREAM("scan_count:"<<scan_count<<" r:"<<r<<" theta:"<<theta*180/M_PI<<"deg");
//    ROS_INFO_STREAM("minRow:"<<minRow<<"  dock_center.point.x:"<<dock_center.point.x<<" dock_center.point.y:"<<dock_center.point.y);


}

double LidarAlign::computeEuclideanDistance(geometry_msgs::msg::PointStamped& x1, geometry_msgs::msg::PointStamped& x2) {
    double d;
    d = sqrt(pow(x1.point.x - x2.point.x, 2) + pow(x1.point.y - x2.point.y, 2));
    return d;
}
/**
 * 求直线ax+by+c=0的参数a，b,c
 * */
void LidarAlign::getLinePara(geometry_msgs::msg::PointStamped& x1, geometry_msgs::msg::PointStamped& x2, LinePara& lp){
    lp.a = x1.point.y - x2.point.y;
    lp.b = x2.point.x - x1.point.x;
    lp.c = x1.point.x*x2.point.y - x2.point.x*x1.point.y;
}
/**
 * 求两条直线的交点
 * **/
void LidarAlign::getCrossPoint(LinePara& para1, LinePara& para2, geometry_msgs::msg::PointStamped& cp) {
    double D;
    D = para1.a*para2.b - para2.a*para1.b;
    cp.point.x = (para1.b*para2.c - para2.b*para1.c)/D;
    cp.point.y = (para2.a*para1.c - para1.a*para2.c)/D;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarAlign>());
  rclcpp::shutdown();
  return 0;
}
