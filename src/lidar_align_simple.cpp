#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "geometry_msgs/msg/point.hpp"
#include <vector>
#include <cmath>
#include <iostream>
#include <array>

#define VISUAL 1
#define MAX_RANGES_FROM_DOCK 1.5
// #define MIN_RANGES_INDEX 60
// #define MAX_RANGES_INDEX 300

constexpr float SMOOTH_THRESHOLD = 0.02f;
constexpr float MUTATION_THRESHOLD_LOW = 0.027f; //dock_thick 0.037
constexpr float MUTATION_THRESHOLD_HIGH = 0.047f;
constexpr float MUTATION_DIFF_THRESHOLD = 0.08f;
constexpr float VISUAL_SCALE = 0.04f;

using namespace std;

class LidarAlignSimpleNode : public rclcpp::Node {
public:
    LidarAlignSimpleNode() : Node("lidar_align_simple") {
        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 10, std::bind(&LidarAlignSimpleNode::laserScanCallback, this, std::placeholders::_1));
        publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("dock", 10);
#if VISUAL
        dock_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("dock_marker", 10);
        point_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("point_marker", 10);
#endif
    }

private:
    void publishDockInfo(float angle_left, float range_left, float angle_right, float range_right, float angle_centre, float range_centre) {
        auto dock_info = std_msgs::msg::Float32MultiArray();
        dock_info.data = {angle_left, range_left, angle_right, range_right, angle_centre, range_centre};
        publisher_->publish(dock_info);
    }

#if VISUAL
    void publishDockVisual(const sensor_msgs::msg::LaserScan::SharedPtr& scan,                         
                              float angle_left, float range_left,
                              float angle_right, float range_right,
                              float angle_centre, float range_centre) {
        geometry_msgs::msg::Point point1, point2, point3;
        point1.x = range_left * cosf(angle_left);
        point1.y = range_left * sinf(angle_left);
        point1.z = 0.0;

        point2.x = range_right * cosf(angle_right);
        point2.y = range_right * sinf(angle_right);
        point2.z = 0.0;

        point3.x = range_centre * cosf(angle_centre);
        point3.y = range_centre * sinf(angle_centre);
        point3.z = 0.0;

        auto dock_hub_mark = visualization_msgs::msg::Marker();
        dock_hub_mark.header.stamp = scan->header.stamp;
        dock_hub_mark.header.frame_id = scan->header.frame_id;
        dock_hub_mark.type = visualization_msgs::msg::Marker::POINTS;
        dock_hub_mark.points = {point1, point2, point3};
        dock_hub_mark.scale.x = VISUAL_SCALE;
        dock_hub_mark.scale.y = VISUAL_SCALE;
        dock_hub_mark.color.r = 154.0 / 255.0;
        dock_hub_mark.color.g = 50.0 / 255.0;
        dock_hub_mark.color.b = 205.0 / 255.0;
        dock_hub_mark.color.a = 1.0;
        dock_marker_pub_->publish(dock_hub_mark);
    }

    void publishMutationVisual(const sensor_msgs::msg::LaserScan::SharedPtr& scan,
                            const std::vector<geometry_msgs::msg::Point>& points){
        auto point_mark = visualization_msgs::msg::Marker();
        point_mark.header.stamp = scan->header.stamp;
        point_mark.header.frame_id = scan->header.frame_id;
        point_mark.type = visualization_msgs::msg::Marker::POINTS;
        point_mark.points = points;
        point_mark.scale.x = VISUAL_SCALE;
        point_mark.scale.y = VISUAL_SCALE;
        point_mark.color.r = 50.0 / 255.0;
        point_mark.color.g = 200.0 / 255.0;
        point_mark.color.b = 150.0 / 255.0;
        point_mark.color.a = 1.0;
        point_marker_pub_->publish(point_mark);
    }
#endif

    void laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
        RCLCPP_INFO(this->get_logger(), "laserScanCallback start...");

        scan_count = scan->ranges.size();
        min_ranges_index = (int) scan_count*1/6;
        max_ranges_index = (int) scan_count*5/6;

        vector<float> laser_ranges(max_ranges_index - min_ranges_index);
        vector<bool> mutation_point(laser_ranges.size(), false);
        array<int, 4> mutation_point_index = {0};
        vector<geometry_msgs::msg::Point> points;

        // Preprocess laser ranges
        for (size_t i = min_ranges_index; i < max_ranges_index; i++) {
            r = scan->ranges[i];
            if (r > MAX_RANGES_FROM_DOCK || isinf(r) || std::isnan(r)) {
                laser_ranges[i - min_ranges_index] = 0;
            } else {
                laser_ranges[i - min_ranges_index] = r;
            }
        }

        // Smooth laser ranges
        for (size_t i = 1; i < laser_ranges.size() - 2; i++) {
            if (laser_ranges[i] < 0.0000001f && (fabs(laser_ranges[i + 1] - laser_ranges[i - 1]) < SMOOTH_THRESHOLD)) {
                laser_ranges[i] = (laser_ranges[i + 1] + laser_ranges[i - 1]) / 2.0f;
            } else if (laser_ranges[i] < 0.0000001f && laser_ranges[i + 1] < 0.0000001f &&
                       (fabs(laser_ranges[i + 2] - laser_ranges[i - 1]) < SMOOTH_THRESHOLD)) {
                laser_ranges[i] = laser_ranges[i + 1] = (laser_ranges[i + 2] + laser_ranges[i - 1]) / 2.0f;
            }
        }

        // Detect mutation points
        int mutation_point_num = 0;
        for (size_t i = 0; i < laser_ranges.size() - 2; i++) {
            float diff = fabs(laser_ranges[i] - laser_ranges[i + 3]);
            if (diff > MUTATION_THRESHOLD_LOW && diff < MUTATION_THRESHOLD_HIGH) {
                mutation_point_num++;
                mutation_point[i] = true;
            }
        }

#if VISUAL
        // Prepare points for visualization
        for (size_t i = 0; i < mutation_point.size(); i++) {
            if (mutation_point[i]) {
                geometry_msgs::msg::Point temp_point;
                temp_point.x = laser_ranges[i] * cosf(scan->angle_min + (i + min_ranges_index) * scan->angle_increment);
                temp_point.y = laser_ranges[i] * sinf(scan->angle_min + (i + min_ranges_index) * scan->angle_increment);
                temp_point.z = 0.0;
                points.push_back(temp_point);
            }
        }
#endif

        // Collect mutation point indices
        int mutation_point_count = 0;
        for (size_t i = 0; i < mutation_point.size(); i++) {
            if (mutation_point[i]) {
                mutation_point_index[mutation_point_count] = i;
                mutation_point_count++;
            }
            if (mutation_point_count == 4) break;
        }

        if (mutation_point_count == 4) {
            bool find_mutation_flag = false;
            for (size_t i = 0; i < mutation_point.size(); i++) {

                if (laser_ranges[mutation_point_index[0]] < laser_ranges[mutation_point_index[1]] &&
                    laser_ranges[mutation_point_index[2]] < laser_ranges[mutation_point_index[3]]) {

                    if (laser_ranges[mutation_point_index[0]] < laser_ranges[mutation_point_index[0] - 5] &&
                        laser_ranges[mutation_point_index[3]] < laser_ranges[mutation_point_index[3] + 5]) {

                        if (abs(mutation_point_index[0] - mutation_point_index[1]) < 7 &&
                            abs(mutation_point_index[2] - mutation_point_index[3]) < 7) {

                            if (fabs(laser_ranges[mutation_point_index[0]] - laser_ranges[mutation_point_index[1]]) < MUTATION_DIFF_THRESHOLD &&
                                fabs(laser_ranges[mutation_point_index[2]] - laser_ranges[mutation_point_index[3]]) < MUTATION_DIFF_THRESHOLD) {
                                    
                                find_mutation_flag = true;

                                int angle_right_dock_index = min_ranges_index + mutation_point_index[0];
                                int angle_left_dock_index = min_ranges_index + mutation_point_index[3];
                                float angle_right_dock = scan->angle_min + scan->angle_increment * angle_right_dock_index;
                                float angle_left_dock = scan->angle_min + scan->angle_increment * angle_left_dock_index;
                                float ranges_right_dock = laser_ranges[mutation_point_index[0]];
                                float ranges_left_dock = laser_ranges[mutation_point_index[3]];
                                
                                float angle_centre_dock = (angle_left_dock + angle_right_dock) / 2.0f;
                                float ranges_centre_dock = scan->ranges[(int)((angle_right_dock_index + angle_left_dock_index)/2)];

                                publishDockInfo(angle_left_dock, ranges_left_dock, angle_right_dock, ranges_right_dock, angle_centre_dock, ranges_centre_dock);
                                // publishMutationVisual(scan, points);

#if VISUAL
                                publishDockVisual(scan, angle_left_dock, ranges_left_dock, angle_right_dock, ranges_right_dock, angle_centre_dock, ranges_centre_dock);
#endif
                                break;
                            }
                        }
                    }
                }

                // Shift mutation point indices
                mutation_point_index[0] = mutation_point_index[1];
                mutation_point_index[1] = mutation_point_index[2];
                mutation_point_index[2] = mutation_point_index[3];

                for (size_t j = mutation_point_index[3] + 1; j < mutation_point.size(); j++) {
                    if (mutation_point[j]) {
                        mutation_point_index[3] = j;
                        break;
                    }
                }
            }

            if (!find_mutation_flag) {
                publishDockInfo(0, 0, 0, 0, 0, 0);
            }
            publishMutationVisual(scan, points);
        } else {
            publishDockInfo(0, 0, 0, 0, 0, 0);
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_;
    int scan_count;
    int min_ranges_index, max_ranges_index;
    double r;
#if VISUAL
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr dock_marker_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr point_marker_pub_;
#endif
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarAlignSimpleNode>());
    rclcpp::shutdown();
    return 0;
}