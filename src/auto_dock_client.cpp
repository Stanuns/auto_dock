#include <functional>
#include <future>
#include <memory>
#include <string>
#include <sstream>

#include "robot_interfaces/action/auto_dock.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "auto_dock/auto_dock_server.hpp"

namespace auto_dock
{
class AutoDockActionClient : public rclcpp::Node
{
public:
  using AutoDock = robot_interfaces::action::AutoDock;
  using GoalHandleAutoDock = rclcpp_action::ClientGoalHandle<AutoDock>;

  explicit AutoDockActionClient(const rclcpp::NodeOptions & options)
  : Node("auto_dock_action_client", options)
  {
    this->client_ptr_ = rclcpp_action::create_client<AutoDock>(
      this,
      "auto_dock");

    // this->timer_ = this->create_wall_timer(
    //   std::chrono::milliseconds(500),
    //   std::bind(&AutoDockActionClient::send_goal, this));
    send_goal();
  }

  void send_goal()
  {
    using namespace std::placeholders;

    // this->timer_->cancel();

    if (!this->client_ptr_->wait_for_action_server()) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      rclcpp::shutdown();
    }

    auto goal_msg = AutoDock::Goal();
    goal_msg.req_state = CHARGED;

    RCLCPP_INFO(this->get_logger(), "Sending goal");

    auto send_goal_options = rclcpp_action::Client<AutoDock>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      std::bind(&AutoDockActionClient::goal_response_callback, this, _1);
    send_goal_options.feedback_callback =
      std::bind(&AutoDockActionClient::feedback_callback, this, _1, _2);
    send_goal_options.result_callback =
      std::bind(&AutoDockActionClient::result_callback, this, _1);
    this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }

private:
  rclcpp_action::Client<AutoDock>::SharedPtr client_ptr_;
//   rclcpp::TimerBase::SharedPtr timer_;

  void goal_response_callback(const GoalHandleAutoDock::SharedPtr & goal_handle)
  {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
  }

  void feedback_callback(
    GoalHandleAutoDock::SharedPtr,
    const std::shared_ptr<const AutoDock::Feedback> feedback)
  {
    std::stringstream ss;
    ss << "Feedback: DockDrive:" << feedback->curr_state << " ";
    RCLCPP_INFO(this->get_logger(), ss.str().c_str());
  }

  void result_callback(const GoalHandleAutoDock::WrappedResult & result)
  {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
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
    std::stringstream ss;
    // ss << "Result received: ";
    // for (auto number : result.result->sequence) {
    //   ss << number << " ";
    // }
    ss << "Result received: " << result.result->res_state << " ";
    RCLCPP_INFO(this->get_logger(), ss.str().c_str());
    rclcpp::shutdown();
  }
};  // class AutoDockActionClient

}  // namespace auto_dock

RCLCPP_COMPONENTS_REGISTER_NODE(auto_dock::AutoDockActionClient)