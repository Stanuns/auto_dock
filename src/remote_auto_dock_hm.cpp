/*
 * 对于华麦底盘，在进行自动对接充电桩后，任意定位点返回充电桩对接充电, 流程节点整合
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
#include "std_msgs/msg/u_int8.hpp"

namespace auto_dock
{
class RemoteAutoDockHm : public rclcpp::Node
{
public:

    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    RemoteAutoDockHm(): 
    Node("remote_auto_dock_hm"),
    send_goal_thread_1(nullptr)
    {
        is_near_dock_pub_ = this->create_publisher<std_msgs::msg::UInt8>("/is_near_dock", 10);
        remote_auto_dock_trigger_sub_ = this->create_subscription<std_msgs::msg::UInt8>(
            "/android_voice_dock", 10, std::bind(&RemoteAutoDockHm::handle_remote_auto_dock_trigger, this, std::placeholders::_1));

        this->nav2_client_ptr_ = rclcpp_action::create_client<NavigateToPose>(
        this,
        "navigate_to_pose");

        RCLCPP_INFO(this->get_logger(), "before nav2_send_goal");

        send_goal_thread_1 = std::make_shared<std::thread>
            (std::bind(&RemoteAutoDockHm::nav2_send_goal, this));
        RCLCPP_INFO(this->get_logger(), "after nav2_send_goal");
    }

    ~RemoteAutoDockHm(){
        // RCLCPP_INFO(this->get_logger(), "~RemoteAutoDockHm()");
        if(send_goal_thread_1){
            send_goal_thread_1->join();
            // send_goal_thread_1.reset();
        }

    }

    void nav2_send_goal()
    {
        while(!remote_auto_dock_trigger_tag){
            sleep(1);
            RCLCPP_INFO(this->get_logger(), "no remote auto dock trigger");
        }

        using namespace std::placeholders;

        if (!this->nav2_client_ptr_->wait_for_action_server(std::chrono::seconds(5))) {
        RCLCPP_ERROR(this->get_logger(), "Navigation2 Action server not available after waiting");
        rclcpp::shutdown();
        }

        auto goal_msg = NavigateToPose::Goal();
        goal_msg.pose.pose.position.x = 0.388556f;
        goal_msg.pose.pose.position.y = 0.949878f;
        goal_msg.pose.pose.orientation.z = 0.585572f;
        goal_msg.pose.pose.orientation.w = 0.810621f;
        goal_msg.pose.header.frame_id = "map";

        RCLCPP_INFO(this->get_logger(), "navigation2 Sending goal");

        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        send_goal_options.goal_response_callback =
        std::bind(&RemoteAutoDockHm::nav2_goal_response_callback, this, _1);
        send_goal_options.feedback_callback =
        std::bind(&RemoteAutoDockHm::nav2_feedback_callback, this, _1, _2);
        send_goal_options.result_callback =
        std::bind(&RemoteAutoDockHm::nav2_result_callback, this, _1);
        this->nav2_client_ptr_->async_send_goal(goal_msg, send_goal_options);
        // RCLCPP_INFO(this->get_logger(), "navigation2 Sent goal");
    }

private:
    rclcpp_action::Client<NavigateToPose>::SharedPtr nav2_client_ptr_;
    // bool IsNearDock = false;
    std::shared_ptr<std::thread> send_goal_thread_1;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr is_near_dock_pub_;
    std_msgs::msg::UInt8 is_near_dock_msg;
    bool remote_auto_dock_trigger_tag = false;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr remote_auto_dock_trigger_sub_;

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

        is_near_dock_msg.data = 0; 
        is_near_dock_pub_->publish(is_near_dock_msg);
    }
    
    void nav2_result_callback(const GoalHandleNavigateToPose::WrappedResult & result)
    {
        switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(get_logger(), "Navigation2 Goal reached");

            // dock_send_goal();
            // IsNearDock = true;
            //发布到topic /is_near_dock 1  与 huamei base_driver联动
            is_near_dock_msg.data = 1;  // 设置 UInt8 数据
            for (int ii=0; ii<3; ii++){
                is_near_dock_pub_->publish(is_near_dock_msg);
            }
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

    void handle_remote_auto_dock_trigger(const std_msgs::msg::UInt8::ConstSharedPtr msg){
        if(msg->data == 2){
            remote_auto_dock_trigger_tag = true;
        }else{
            remote_auto_dock_trigger_tag = false;
        }     
    }

};  // class RemoteAutoDockHm
} //namespace auto_dock

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<auto_dock::RemoteAutoDockHm>();
//   node->nav2_send_goal();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}