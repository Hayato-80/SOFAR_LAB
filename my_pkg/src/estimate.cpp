// this node has its class fully defined in the cpp file
// for larger nodes, the class can be classically split in header / source


// include any thing required - do not forget to use the .hpp extension for ROS 2 files
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <turtlesim/msg/pose.hpp>
#include "tf2/LinearMath/Quaternion.h"
#include <algorithm>

#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include "tf2/LinearMath/Quaternion.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <my_interfaces/srv/get_turtle_pose.hpp>

// #include <my_msgs/msg/my_output_msg.hpp>


using namespace std::chrono_literals;

namespace my_pkg
{

class lab2Node : public rclcpp::Node
{
public:
    lab2Node(rclcpp::NodeOptions options) : Node("lab2_node", options), tf_buffer_(this->get_clock(), tf2::durationFromSec(10.0)), tf_listener_(tf_buffer_)
    {

        odom_subscription = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&lab2Node::compute_and_publish_cmd_vel, this, std::placeholders::_1));
        
        goal_subscription = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal_pose", 10, std::bind(&lab2Node::goal_callback, this, std::placeholders::_1));
        
        cmd_vel_publisher = create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);

        this->declare_parameter<float>("v", 0.1);
        this->declare_parameter<float>("delta_t", 0.1);

        turtle_pose_client_ = this->create_client<my_interfaces::srv::GetTurtlePose>("get_turtle_pose");
        
    }
  
private:

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_subscription;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher;
    geometry_msgs::msg::Pose current_pose;
    geometry_msgs::msg::Pose goal_pose;
    float x_other;
    float y_other;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    rclcpp::Client<my_interfaces::srv::GetTurtlePose>::SharedPtr turtle_pose_client_;

    void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr goal_msg)
    {
        RCLCPP_INFO(this->get_logger(), "Goal callback triggered");

        geometry_msgs::msg::TransformStamped transform_stamped = tf_buffer_.lookupTransform("odom", goal_msg->header.frame_id, tf2::TimePointZero);
        geometry_msgs::msg::PoseStamped tranformed_goal_frame;
        tf2::doTransform(*goal_msg, tranformed_goal_frame, transform_stamped);
        
        try
        
        goal_pose = tranformed_goal_frame.pose;

        
    }

    void compute_and_publish_cmd_vel(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        std::vector<std::pair<float, float>> other_robot_pose;

        current_pose.position.x = msg->pose.pose.position.x;
        current_pose.position.y = msg->pose.pose.position.y;
        current_pose.orientation = msg->pose.pose.orientation;

        float x = current_pose.position.x;
        float y = current_pose.position.y;
        tf2::Quaternion q(
            current_pose.orientation.x,
            current_pose.orientation.y,
            current_pose.orientation.z,
            current_pose.orientation.w);
        
        tf2::Matrix3x3 rpy(q);
        double roll, pitch, yaw;
        rpy.getRPY(roll, pitch, yaw);
        float theta = yaw;
        
        float optimal_yaw_rate = 0.0;
        float v = this->get_parameter("v").as_double();
        float delta_t = this->get_parameter("delta_t").as_double();

        auto request = std::make_shared<my_interfaces::srv::GetTurtlePose::Request>();
        request->turtle_name = "turtle2";
        

        if (!turtle_pose_client_->wait_for_service(1s)) {
            RCLCPP_WARN(this->get_logger(), "Service not available, skipping collision check.");
        } else {
            auto request = std::make_shared<my_interfaces::srv::GetTurtlePose::Request>();
            request->turtle_name = "turtle2";
        
            // Call the service synchronously
            try {
                auto response = turtle_pose_client_->async_send_request(request).get();
                x_other = response->x;
                y_other = response->y;
                //other_robot_pose.emplace_back(response->x, response->y); // Add the response to the vector
            } catch (const std::exception &e) {
                RCLCPP_ERROR(this->get_logger(), "Service call failed: %s", e.what());
            }
        }

        
        //auto other_robot_pose.emplace_back(request->x, request->y);
        
        float min_distance = 1000.0;
        // (float w = -3.14; w < 3.14; w += 0.001)
        for (float w = -1.0; w < 1.0; w += 0.001) {
            float x_pred = x + v*delta_t*cos(theta+w*delta_t/2);
            float y_pred = y + v*delta_t*sin(theta+w*delta_t/2);
            float distance = sqrt(pow(x_pred - goal_pose.position.x, 2) + pow(y_pred - goal_pose.position.y, 2));
            
            bool close_to_other = false;
            for (const auto& [other_x, other_y] : other_robot_pose) {
                float distance_to_other = sqrt(pow(x_pred - other_x, 2) + pow(y_pred - other_y, 2));
                if(distance_to_other < 0.01){
                    close_to_other = true;
                    break;
                }
            }

            if (close_to_other) {
                continue; // Skip this yaw rate if too close to another robot
            }

            if (distance < min_distance) {
                min_distance = distance;
                optimal_yaw_rate  = w;
                //optimal_yaw_rate  = w/delta_t;
                //optimal_yaw_rate  = angle_diff / delta_t;
                RCLCPP_INFO(this->get_logger(), "Optimal yaw rate updated: %f", optimal_yaw_rate);
            }

        }
        RCLCPP_INFO(this->get_logger(), "Current goal_pose: x=%f, y=%f", goal_pose.position.x, goal_pose.position.y);

        
        // if(goal_pose.position.x == 0 && goal_pose.position.y == 0){
        //     geometry_msgs::msg::Twist cmd_vel_msg;
        //     cmd_vel_msg.linear.x = 0.0;
        //     cmd_vel_msg.angular.z = 0.0;
        //     cmd_vel_publisher->publish(cmd_vel_msg);
            
        //     RCLCPP_INFO(this->get_logger(), "Goal is not set");
        //     return;
        // }
        // else {
        if (min_distance < 0.1) {
            geometry_msgs::msg::Twist cmd_vel_msg;
            cmd_vel_msg.linear.x = 0.0;
            cmd_vel_msg.angular.z = 0.0;
            cmd_vel_publisher->publish(cmd_vel_msg);
            
            RCLCPP_INFO(this->get_logger(), "Goal reached");
            return;
        }
        else{
            geometry_msgs::msg::Twist cmd_vel_msg;
            cmd_vel_msg.linear.x = v;
            cmd_vel_msg.angular.z = optimal_yaw_rate;
            cmd_vel_publisher->publish(cmd_vel_msg);
            RCLCPP_INFO(this->get_logger(), "Published cmd_vel: linear.x=%f, angular.z=%f", cmd_vel_msg.linear.x, cmd_vel_msg.angular.z);
        }

        //}
        
    }
};

}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  rclcpp::spin(std::make_shared<my_pkg::lab2Node>(options));
  rclcpp::shutdown();
  return 0;
}

//Limitations
// Assumes constant velocity v and fixed time Δt, not very realistic.
// No obstacle dynamics — doesn’t account for future movement of other robots.
// Local planner only — no global planning or path memory.
// Collision avoidance is binary — just removes close predictions rather than optimizing a path around them.

// Improvements
// Dynamic prediction — Predict future positions of other robots.
// Cost function optimization — use full cost map instead of distance-only.
// Global planner — integrate a Dijkstra/A* based planner.
// Use occupancy grid — enhance environment awareness.
// Use a smoother control model — instead of discrete yaw options, consider gradient optimization over ω.