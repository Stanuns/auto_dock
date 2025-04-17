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
#include "geometry_msgs/msg/twist.hpp"

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
        remote_auto_dock_trigger_tag = false;

        is_near_dock_pub_ = this->create_publisher<std_msgs::msg::UInt8>("/is_near_dock", 10);
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
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
        rclcpp::shutdown();

    }

    void nav2_send_goal()
    {
        while(rclcpp::ok()){
            while(!remote_auto_dock_trigger_tag){
                sleep(1);
                RCLCPP_INFO(this->get_logger(), "no remote auto dock trigger");
            }
    
            using namespace std::placeholders;
    
            if (!this->nav2_client_ptr_->wait_for_action_server(std::chrono::seconds(5))) {
                RCLCPP_ERROR(this->get_logger(), "Navigation2 Action server not available after waiting");
                // rclcpp::shutdown();
                return;
            }
    
            auto goal_msg = NavigateToPose::Goal();
            goal_msg.pose.pose.position.x = 0.59847f;
            goal_msg.pose.pose.position.y = 1.10564f;
            goal_msg.pose.pose.orientation.z = 0.662676f;
            goal_msg.pose.pose.orientation.w = 0.748906f;
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

            remote_auto_dock_trigger_tag = false;
        }
        
    }

private:
    rclcpp_action::Client<NavigateToPose>::SharedPtr nav2_client_ptr_;
    // bool IsNearDock = false;
    std::shared_ptr<std::thread> send_goal_thread_1;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr is_near_dock_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    std_msgs::msg::UInt8 is_near_dock_msg;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr remote_auto_dock_trigger_sub_;
    bool remote_auto_dock_trigger_tag;

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
        // RCLCPP_INFO(get_logger(), "Distance remaining: %f", feedback->distance_remaining);

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
            RCLCPP_INFO(this->get_logger(), "Navigation2 nav2_result_callback is done");
            break;
            // return;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(), "Navigation2 Goal was aborted");
            break;
            // return;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR(this->get_logger(), "Navigation2 Goal was canceled");
            break;
            // return;
        default:
            RCLCPP_ERROR(this->get_logger(), "Unknown result code");
            break;
            // return;
        }
        // rclcpp::shutdown();

        // Call nav2_send_goal() again after the result is processed
        // std::thread([this]() {
        //     nav2_send_goal();
        // }).detach();
        // remote_auto_dock_trigger_tag = false;
        // auto future = std::async(std::launch::async, [this]() {
        //     nav2_send_goal();
        //   });
        geometry_msgs::msg::Twist msg;
        msg.linear.x = 0.0;
        msg.angular.z = 0.0;
        for(int ii=0; ii < 10; ii++){
            cmd_vel_pub_->publish(msg);
            sleep(0.01);
        }

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