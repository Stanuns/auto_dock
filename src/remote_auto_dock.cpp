/*
 * @Author: Wei Sun 
 * @Date: 2025-01-07 14:09:42 
 * @Last Modified by: Wei Sun
 * @Last Modified time: 2025-01-10 16:20:45
 */
/*
 * 任意定位点返回充电桩对接充电, 流程节点整合
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
#include "robot_interfaces/action/auto_dock.hpp"
#include "auto_dock/auto_dock_server.hpp"

namespace auto_dock
{
class RemoteAutoDock : public rclcpp::Node
{
public:

    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    using AutoDock = robot_interfaces::action::AutoDock;
    using GoalHandleAutoDock = rclcpp_action::ClientGoalHandle<AutoDock>;

  RemoteAutoDock(): 
  Node("remote_auto_dock"),
  send_goal_thread_1(nullptr),
  send_goal_thread_2(nullptr)
  {
    this->nav2_client_ptr_ = rclcpp_action::create_client<NavigateToPose>(
      this,
      "navigate_to_pose");
    this->dock_client_ptr_ = rclcpp_action::create_client<AutoDock>(
      this,
      "auto_dock");

    RCLCPP_INFO(this->get_logger(), "before nav2_send_goal dock_send_goal");
    // std::bind(&RemoteAutoDock::nav2_send_goal, this);
    // std::bind(&RemoteAutoDock::dock_send_goal, this);
    // nav2_send_goal();
    // dock_send_goal();
    send_goal_thread_1 = std::make_shared<std::thread>
        (std::bind(&RemoteAutoDock::nav2_send_goal, this));
    send_goal_thread_2 = std::make_shared<std::thread>
        (std::bind(&RemoteAutoDock::dock_send_goal, this));
    RCLCPP_INFO(this->get_logger(), "after nav2_send_goal dock_send_goal");
  }

  ~RemoteAutoDock(){
    // RCLCPP_INFO(this->get_logger(), "~RemoteAutoDock()");
    if(send_goal_thread_1){
        send_goal_thread_1->join();
        // send_goal_thread_1.reset();
    }

    if(send_goal_thread_2){
        send_goal_thread_2->join();
        // send_goal_thread_2.reset();
    }

  }

  void nav2_send_goal()
  {
    using namespace std::placeholders;

    if (!this->nav2_client_ptr_->wait_for_action_server(std::chrono::seconds(5))) {
      RCLCPP_ERROR(this->get_logger(), "Navigation2 Action server not available after waiting");
      rclcpp::shutdown();
    }

    auto goal_msg = NavigateToPose::Goal();
    goal_msg.pose.pose.position.x = 0.0f;
    goal_msg.pose.pose.position.y = 0.0f;
    // goal_msg.pose.pose.orientation.w = 1.0f;
    goal_msg.pose.header.frame_id = "map";

    RCLCPP_INFO(this->get_logger(), "navigation2 Sending goal");

    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      std::bind(&RemoteAutoDock::nav2_goal_response_callback, this, _1);
    send_goal_options.feedback_callback =
      std::bind(&RemoteAutoDock::nav2_feedback_callback, this, _1, _2);
    send_goal_options.result_callback =
      std::bind(&RemoteAutoDock::nav2_result_callback, this, _1);
    this->nav2_client_ptr_->async_send_goal(goal_msg, send_goal_options);
    // RCLCPP_INFO(this->get_logger(), "navigation2 Sent goal");
  }

  void dock_send_goal()
  {
    while(!IsNearDock){
      sleep(1);
      RCLCPP_INFO(this->get_logger(), "dock_send_goal IsNearDock = %d",IsNearDock);
    }

    using namespace std::placeholders;

    // this->timer_->cancel();

    if (!this->dock_client_ptr_->wait_for_action_server()) {
      RCLCPP_ERROR(this->get_logger(), "Auto Dock Action server not available after waiting");
      rclcpp::shutdown();
    }

    auto goal_msg = AutoDock::Goal();
    goal_msg.req_state = CHARGED;

    RCLCPP_INFO(this->get_logger(), "Auto Dock Sending goal");

    auto send_goal_options = rclcpp_action::Client<AutoDock>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      std::bind(&RemoteAutoDock::dock_goal_response_callback, this, _1);
    send_goal_options.feedback_callback =
      std::bind(&RemoteAutoDock::dock_feedback_callback, this, _1, _2);
    send_goal_options.result_callback =
      std::bind(&RemoteAutoDock::dock_result_callback, this, _1);
    this->dock_client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }

private:
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav2_client_ptr_;
  rclcpp_action::Client<AutoDock>::SharedPtr dock_client_ptr_;
  bool IsNearDock = false;
  std::shared_ptr<std::thread> send_goal_thread_1;
  std::shared_ptr<std::thread> send_goal_thread_2;

  //navigation2
  void nav2_goal_response_callback(const GoalHandleNavigateToPose::SharedPtr & goal_handle)
  {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Navigation2 Goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(), "Navigation2 Goal accepted by server, waiting for result");
    }
  }

  void nav2_feedback_callback(
    GoalHandleNavigateToPose::SharedPtr,
    const std::shared_ptr<const NavigateToPose::Feedback> feedback)
  {
    RCLCPP_INFO(get_logger(), "Distance remaining: %f", feedback->distance_remaining);
  }
  
  void nav2_result_callback(const GoalHandleNavigateToPose::WrappedResult & result)
  {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(get_logger(), "Navigation2 Goal reached");

        // dock_send_goal();
        IsNearDock = true;
        break;
        // return;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Navigation2 Goal was aborted");
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Navigation2 Goal was canceled");
        return;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        return;
    }
    // rclcpp::shutdown();
    RCLCPP_INFO(this->get_logger(), "Navigation2 nav2_result_callback is done");
  }

  //auto dock
  void dock_goal_response_callback(const GoalHandleAutoDock::SharedPtr & goal_handle)
  {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Auto Dock Goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(), "Auto Dock Goal accepted by server, waiting for result");
    }
  }

  void dock_feedback_callback(
    GoalHandleAutoDock::SharedPtr,
    const std::shared_ptr<const AutoDock::Feedback> feedback)
  {
    std::stringstream ss;
    ss << "Feedback: DockDrive:" << feedback->curr_state << " ";
    RCLCPP_INFO(this->get_logger(), ss.str().c_str());
  }

  void dock_result_callback(const GoalHandleAutoDock::WrappedResult & result)
  {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Auto Dock Goal was aborted");
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Auto Dock Goal was canceled");
        return;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        return;
    }
    IsNearDock = false;
    std::stringstream ss;
    // ss << "Result received: ";
    // for (auto number : result.result->sequence) {
    //   ss << number << " ";
    // }
    ss << "Result received: " << result.result->res_state << " ";
    RCLCPP_INFO(this->get_logger(), ss.str().c_str());
    rclcpp::shutdown();
  }


};  // class RemoteAutoDock
} //namespace auto_dock

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<auto_dock::RemoteAutoDock>();
//   node->nav2_send_goal();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}