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
class NavToGoalRecycle : public rclcpp::Node
{
public:

    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    NavToGoalRecycle(): 
    Node("nav_to_goal_recycle"),
    send_goal_thread_1(nullptr)
    {
        nav_to_next_goal_tag = true;
        node_exec = true;
        count = 0;
        count_direction = 1;
        is_near_dock_pub_ = this->create_publisher<std_msgs::msg::UInt8>("/is_near_dock", 10);
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        nav_to_goal_recycle_sub_ = this->create_subscription<std_msgs::msg::UInt8>(
            "/nav_to_goal_recycle_trigger", 10, std::bind(&NavToGoalRecycle::handle_nav_to_goal_recycle_trigger, this, std::placeholders::_1));

        this->nav2_client_ptr_ = rclcpp_action::create_client<NavigateToPose>(
        this,
        "navigate_to_pose");

        RCLCPP_INFO(this->get_logger(), "before nav2_send_goal");

        send_goal_thread_1 = std::make_shared<std::thread>
            (std::bind(&NavToGoalRecycle::nav2_send_goal, this));
        RCLCPP_INFO(this->get_logger(), "after nav2_send_goal");
    }

    ~NavToGoalRecycle(){
        // RCLCPP_INFO(this->get_logger(), "~NavToGoalRecycle()");
        if(send_goal_thread_1){
            send_goal_thread_1->join();
            // send_goal_thread_1.reset();
        }
        rclcpp::shutdown();

    }

    void nav2_send_goal()
    {
        while(rclcpp::ok() && node_exec){
            while(!nav_to_next_goal_tag && node_exec){
                sleep(1);
                RCLCPP_INFO(this->get_logger(), "Do not nav to next goal");
            }

            if(node_exec){
                using namespace std::placeholders;

                if (!this->nav2_client_ptr_->wait_for_action_server(std::chrono::seconds(5))) {
                    RCLCPP_ERROR(this->get_logger(), "Navigation2 Action server not available after waiting");
                    // rclcpp::shutdown();
                    return;
                }
    
                auto goal_msg = NavigateToPose::Goal();
                goal_msg.pose.header.frame_id = "map";
                if(count==0){
                    goal_msg.pose.pose.position.x = -1.095f;
                    goal_msg.pose.pose.position.y = -2.0801f;
                    goal_msg.pose.pose.orientation.z = -0.19404f;
                    goal_msg.pose.pose.orientation.w = 0.98099f;
                }else if(count==1){
                    goal_msg.pose.pose.position.x = -0.23927f;
                    goal_msg.pose.pose.position.y = 0.79599f;
                    goal_msg.pose.pose.orientation.z = 0.72072f;
                    goal_msg.pose.pose.orientation.w = 0.69323f;
                }else if(count==2){
                    goal_msg.pose.pose.position.x = 1.8553f;
                    goal_msg.pose.pose.position.y = -0.14975f;
                    goal_msg.pose.pose.orientation.z = -0.71402f; //-0.71402
                    goal_msg.pose.pose.orientation.w = 0.70012f; //0.70012
                }else if(count==3){
                    goal_msg.pose.pose.position.x = 3.0204f;
                    goal_msg.pose.pose.position.y = -2.8312f;
                    goal_msg.pose.pose.orientation.z = 0.99076f;
                    goal_msg.pose.pose.orientation.w = -0.13564f;
                }
                count += count_direction;
                if (count >= 3){
                    count_direction = -1;
                }else if(count <=0 ){
                    count_direction = 1;
                }
                
    
                RCLCPP_INFO(this->get_logger(), "navigation2 Sending goal, count: %d", count);
    
                auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
                send_goal_options.goal_response_callback =
                    std::bind(&NavToGoalRecycle::nav2_goal_response_callback, this, _1);
                send_goal_options.feedback_callback =
                    std::bind(&NavToGoalRecycle::nav2_feedback_callback, this, _1, _2);
                send_goal_options.result_callback =
                    std::bind(&NavToGoalRecycle::nav2_result_callback, this, _1);
                this->nav2_client_ptr_->async_send_goal(goal_msg, send_goal_options);
                // RCLCPP_INFO(this->get_logger(), "navigation2 Sent goal");
            }
            

            nav_to_next_goal_tag = false;
        }
        
    }

private:
    rclcpp_action::Client<NavigateToPose>::SharedPtr nav2_client_ptr_;
    // bool IsNearDock = false;
    std::shared_ptr<std::thread> send_goal_thread_1;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr is_near_dock_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    std_msgs::msg::UInt8 is_near_dock_msg;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr nav_to_goal_recycle_sub_;
    bool nav_to_next_goal_tag, node_exec;
    int count, count_direction;

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
        nav_to_next_goal_tag = true;
        geometry_msgs::msg::Twist msg;
        msg.linear.x = 0.0;
        msg.angular.z = 0.0;
        for(int ii=0; ii < 10; ii++){
            cmd_vel_pub_->publish(msg);
            sleep(0.01);
        }
    }

    void handle_nav_to_goal_recycle_trigger(const std_msgs::msg::UInt8::ConstSharedPtr msg){
        if(msg->data == 1){
            nav_to_next_goal_tag = true;
        }else if(msg->data == 0){
            nav_to_next_goal_tag = false;
            RCLCPP_INFO(this->get_logger(), "topic /nav_to_goal_recycle_trigger nav_to_next_goal_tag is 0, shut down the node");
            node_exec = false;
            rclcpp::shutdown();
        }     
    }

};  // class NavToGoalRecycle
} //namespace auto_dock

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<auto_dock::NavToGoalRecycle>();
//   node->nav2_send_goal();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}