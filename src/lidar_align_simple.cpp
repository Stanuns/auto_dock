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
#define MAX_RANGES_FROM_CHANGE 2.0
#define MIN_RANGES_INDEX 90
#define MAX_RANGES_INDEX 270

constexpr float SMOOTH_THRESHOLD = 0.02f;
constexpr float MUTATION_THRESHOLD_LOW = 0.025f;
constexpr float MUTATION_THRESHOLD_HIGH = 0.065f;
constexpr float MUTATION_DIFF_THRESHOLD = 0.08f;
constexpr float VISUAL_SCALE = 0.04f;

class ChargeHubNode : public rclcpp::Node {
public:
    ChargeHubNode() : Node("charge_hub") {
        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 10, std::bind(&ChargeHubNode::laserScanCallback, this, std::placeholders::_1));
        publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("charge", 10);
#if VISUAL
        charge_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("charge_marker", 10);
        point_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("point_marker", 10);
#endif
    }

private:
    void publishChargeInfo(float angle_left, float range_left, float angle_right, float range_right, float angle_centre, float range_centre) {
        auto charge_info = std_msgs::msg::Float32MultiArray();
        charge_info.data = {angle_left, range_left, angle_right, range_right, angle_centre, range_centre};
        publisher_->publish(charge_info);
    }

#if VISUAL
    void publishVisualization(const sensor_msgs::msg::LaserScan::SharedPtr& msg, 
                              const std::vector<geometry_msgs::msg::Point>& points,
                              float angle_left, float range_left,
                              float angle_right, float range_right,
                              float angle_centre, float range_centre) {
        geometry_msgs::msg::Point point1, point2, point3;
        point1.x = -range_left * cosf(angle_left * msg->angle_increment);
        point1.y = -range_left * sinf(angle_left * msg->angle_increment);
        point1.z = 0.0;

        point2.x = -range_right * cosf(angle_right * msg->angle_increment);
        point2.y = -range_right * sinf(angle_right * msg->angle_increment);
        point2.z = 0.0;

        point3.x = -range_centre * cosf(angle_centre * msg->angle_increment);
        point3.y = -range_centre * sinf(angle_centre * msg->angle_increment);
        point3.z = 0.0;

        auto charge_hub_mark = visualization_msgs::msg::Marker();
        charge_hub_mark.header.stamp = msg->header.stamp;
        charge_hub_mark.header.frame_id = msg->header.frame_id;
        charge_hub_mark.type = visualization_msgs::msg::Marker::POINTS;
        charge_hub_mark.points = {point1, point2, point3};
        charge_hub_mark.scale.x = VISUAL_SCALE;
        charge_hub_mark.scale.y = VISUAL_SCALE;
        charge_hub_mark.color.r = 154.0 / 255.0;
        charge_hub_mark.color.g = 50.0 / 255.0;
        charge_hub_mark.color.b = 205.0 / 255.0;
        charge_hub_mark.color.a = 1.0;
        charge_marker_pub_->publish(charge_hub_mark);

        auto point_mark = visualization_msgs::msg::Marker();
        point_mark.header.stamp = msg->header.stamp;
        point_mark.header.frame_id = msg->header.frame_id;
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

    void laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        std::vector<float> laser_ranges(MAX_RANGES_INDEX - MIN_RANGES_INDEX);
        std::vector<bool> mutation_point(laser_ranges.size(), false);
        std::array<int, 4> mutation_point_index = {0};
        std::vector<geometry_msgs::msg::Point> points;

        // Preprocess laser ranges
        for (size_t i = MIN_RANGES_INDEX; i < MAX_RANGES_INDEX; i++) {
            if (std::isinf(msg->ranges[i]) || msg->ranges[i] > MAX_RANGES_FROM_CHANGE) {
                laser_ranges[i - MIN_RANGES_INDEX] = 0;
            } else {
                laser_ranges[i - MIN_RANGES_INDEX] = msg->ranges[i];
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
            float diff = fabs(laser_ranges[i] - laser_ranges[i + 1]);
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
                temp_point.x = -laser_ranges[i] * cosf((i + MIN_RANGES_INDEX) * msg->angle_increment);
                temp_point.y = -laser_ranges[i] * sinf((i + MIN_RANGES_INDEX) * msg->angle_increment);
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

                                int angle_left_charge = MIN_RANGES_INDEX + mutation_point_index[3];
                                int angle_right_charge = MIN_RANGES_INDEX + mutation_point_index[0];
                                float ranges_left_charge = laser_ranges[mutation_point_index[3]];
                                float ranges_right_charge = laser_ranges[mutation_point_index[0]] + 0.06f;
                                int angle_centre_charge = (int)(0.5f + (angle_left_charge + angle_right_charge) / 2.0f);
                                float ranges_centre_charge = msg->ranges[angle_centre_charge];

                                publishChargeInfo(angle_left_charge, ranges_left_charge, angle_right_charge, ranges_right_charge, angle_centre_charge, ranges_centre_charge);

#if VISUAL
                                publishVisualization(msg, points, angle_left_charge, ranges_left_charge, angle_right_charge, ranges_right_charge, angle_centre_charge, ranges_centre_charge);
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
                publishChargeInfo(0, 0, 0, 0, 0, 0);
            }
        } else {
            publishChargeInfo(0, 0, 0, 0, 0, 0);
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_;
#if VISUAL
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr charge_marker_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr point_marker_pub_;
#endif
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ChargeHubNode>());
    rclcpp::shutdown();
    return 0;
}