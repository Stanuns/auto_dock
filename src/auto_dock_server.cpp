#include <functional>
#include <memory>
#include <thread>
#include <string>

#include "robot_interfaces/action/auto_dock.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "auto_dock/auto_dock_server.hpp"
#include "auto_dock/dock_driver.hpp"
#include "auto_dock/visibility_control.h"

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h> 
// #include <message_filters/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "robot_interfaces/msg/dock_pose_stamped.hpp" 
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"


namespace auto_dock
{
class AutoDockActionServer : public rclcpp::Node
{
public:
  using AutoDock = robot_interfaces::action::AutoDock;
  using GoalHandleAutoDock = rclcpp_action::ServerGoalHandle<AutoDock>;

  message_filters::Subscriber<nav_msgs::msg::Odometry> odom_sub_;
  message_filters::Subscriber<robot_interfaces::msg::DockPoseStamped> relative_dock_pose_sub_;
  typedef message_filters::sync_policies::ApproximateTime<
        nav_msgs::msg::Odometry,
        robot_interfaces::msg::DockPoseStamped
  > SyncPolicy;
  message_filters::Synchronizer<SyncPolicy> sync_;

  // AUTO_DOCK_PUBLIC
  // explicit AutoDockActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  // : Node("auto_dock_action_server", options),
  AUTO_DOCK_PUBLIC
  explicit AutoDockActionServer()
  : Node("auto_dock_action_server"),
  sync_(SyncPolicy(10), odom_sub_, relative_dock_pose_sub_)
  {
    using namespace std::placeholders;

    this->action_server_ = rclcpp_action::create_server<AutoDock>(
      this,
      "auto_dock",
      std::bind(&AutoDockActionServer::handle_goal, this, _1, _2),
      std::bind(&AutoDockActionServer::handle_cancel, this, _1),
      std::bind(&AutoDockActionServer::handle_accepted, this, _1));

    dock_drive_ = new DockDriver();

    odom_sub_.subscribe(this, "/odom_combined");
    relative_dock_pose_sub_.subscribe(this, "/relative_dock_pose");
    sync_.registerCallback(&AutoDockActionServer::syncCallback, this);

    RCLCPP_INFO(this->get_logger(), "AutoDockActionServer initialize done......");
  }

  ~AutoDockActionServer(void){
    delete dock_drive_;
  }

  void syncCallback(const nav_msgs::msg::Odometry::ConstSharedPtr odom, const robot_interfaces::msg::DockPoseStamped::ConstSharedPtr relative_dock_pose){
    RCLCPP_INFO(this->get_logger(), "syncCallback......");
    // make sure that the action hasn't been canceled
    // if(accepted_goal_handle->is_canceling()){
    //   const auto goal = accepted_goal_handle->get_goal();

    //   RCLCPP_INFO(this->get_logger(), "Goal accepted:%d", goal->req_state);
    // }else{
    //   RCLCPP_INFO(this->get_logger(), "Goal unaccepted");
    // }
  }

private:
  rclcpp_action::Server<AutoDock>::SharedPtr action_server_;

  //初始化dock_dirve
  DockDriver* dock_drive_;
  std::shared_ptr<GoalHandleAutoDock> accepted_goal_handle;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const AutoDock::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received auto dock goal request: %d", goal->req_state);
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleAutoDock> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel auto dock goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleAutoDock> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "handle_accepted......");
    // using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    // std::thread{std::bind(&AutoDockActionServer::execute, this, _1), goal_handle}.detach();
    accepted_goal_handle = goal_handle;
    // sync_.registerCallback(&AutoDockActionServer::syncCallback, this);
  }

  void execute(const std::shared_ptr<GoalHandleAutoDock> goal_handle)
  {
    // RCLCPP_INFO(this->get_logger(), "Executing goal");
    // rclcpp::Rate loop_rate(1);
    // const auto goal = goal_handle->get_goal();
    // auto feedback = std::make_shared<Fibonacci::Feedback>();
    // auto & sequence = feedback->partial_sequence;
    // sequence.push_back(0);
    // sequence.push_back(1);
    // auto result = std::make_shared<Fibonacci::Result>();

    // for (int i = 1; (i < goal->order) && rclcpp::ok(); ++i) {
    //   // Check if there is a cancel request
    //   if (goal_handle->is_canceling()) {
    //     result->sequence = sequence;
    //     goal_handle->canceled(result);
    //     RCLCPP_INFO(this->get_logger(), "Goal canceled");
    //     return;
    //   }
    //   // Update sequence
    //   sequence.push_back(sequence[i] + sequence[i - 1]);
    //   // Publish feedback
    //   goal_handle->publish_feedback(feedback);
    //   RCLCPP_INFO(this->get_logger(), "Publish feedback");

    //   loop_rate.sleep();
    // }

    // // Check if goal is done
    // if (rclcpp::ok()) {
    //   result->sequence = sequence;
    //   goal_handle->succeed(result);
    //   RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    // }
  }


};  // class AutoDockActionServer

}  // namespace auto_dock

// RCLCPP_COMPONENTS_REGISTER_NODE(auto_dock::AutoDockActionServer)
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<auto_dock::AutoDockActionServer>());
    rclcpp::shutdown();
    return 0;
}