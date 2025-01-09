/*
 * @Author: Wei Sun 
 * @Date: 2025-01-07 14:09:42 
 * @Last Modified by: Wei Sun
 * @Last Modified time: 2025-01-09 17:45:01
 */
/*
 * 任意定位点返回充电桩对接充电
 */
#include <functional>
#include <future>
#include <memory>
#include <string>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"

namespace auto_dock
{
class RemoteAutoDock : public rclcpp::Node
{
public:

    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  RemoteAutoDock()
  : Node("remote_auto_dock")
  {
    this->client_ptr_ = rclcpp_action::create_client<NavigateToPose>(
      this,
      "navigate_to_pose");

    send_navigation_goal();
  }

  void send_navigation_goal()
  {
    using namespace std::placeholders;

    if (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(5))) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      rclcpp::shutdown();
    }

    auto goal_msg = NavigateToPose::Goal();
    goal_msg.pose.pose.position.x = -4.0f;
    goal_msg.pose.pose.position.y = -2.0f;
    // goal_msg.pose.pose.orientation.w = 1.0f;
    goal_msg.pose.header.frame_id = "map";

    RCLCPP_INFO(this->get_logger(), "Sending goal");

    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      std::bind(&RemoteAutoDock::goal_response_callback, this, _1);
    send_goal_options.feedback_callback =
      std::bind(&RemoteAutoDock::feedback_callback, this, _1, _2);
    send_goal_options.result_callback =
      std::bind(&RemoteAutoDock::result_callback, this, _1);
    this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }

private:
  rclcpp_action::Client<NavigateToPose>::SharedPtr client_ptr_;

  void goal_response_callback(const GoalHandleNavigateToPose::SharedPtr & goal_handle)
  {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
  }

  void feedback_callback(
    GoalHandleNavigateToPose::SharedPtr,
    const std::shared_ptr<const NavigateToPose::Feedback> feedback)
  {
    RCLCPP_INFO(get_logger(), "Distance remaining: %f", feedback->distance_remaining);
  }

  void result_callback(const GoalHandleNavigateToPose::WrappedResult & result)
  {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(get_logger(), "Goal reached");
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
        return;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        return;
    }
    // std::stringstream ss;
    // ss << "Result received: ";
    // for (auto number : result.result->sequence) {
    //   ss << number << " ";
    // }
    // RCLCPP_INFO(this->get_logger(), ss.str().c_str());
    rclcpp::shutdown();
  }
};  // class RemoteAutoDock
} //namespace auto_dock

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<auto_dock::RemoteAutoDock>();
//   node->send_navigation_goal();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}