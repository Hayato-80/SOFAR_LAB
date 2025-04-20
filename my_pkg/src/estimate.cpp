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
#include "tf2/LinearMath/Quaternion.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

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
            "/odom", 10, std::bind(&lab2Node::odom_callback, this, std::placeholders::_1));
        
        goal_subscription = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal_pose", 10, std::bind(&lab2Node::goal_callback, this, std::placeholders::_1));
        
        cmd_vel_publisher = create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);

        this->declare_parameter<float>("v", 0.1);
        this->declare_parameter<float>("delta_t", 0.1);
        
    }
  
private:

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_subscription;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher;
    geometry_msgs::msg::Pose current_pose;
    geometry_msgs::msg::Pose goal_pose;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        //RCLCPP_INFO(this->get_logger(), "Pose callback triggered");
        try{
            geometry_msgs::msg::TransformStamped transform = tf_buffer_.lookupTransform("map", "odom", tf2::TimePointZero);
            current_pose.position.x = msg->pose.pose.position.x + transform.transform.translation.x;
            current_pose.position.y = msg->pose.pose.position.y + transform.transform.translation.y;
            current_pose.orientation = msg->pose.pose.orientation;
        }
        catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "Could not transform base_link to map: %s", ex.what());
        }
        
    }

    void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr goal_msg)
    {
        RCLCPP_INFO(this->get_logger(), "Goal callback triggered");
        goal_pose = goal_msg->pose; 
        compute_and_publish_cmd_vel();
    }

    void compute_and_publish_cmd_vel()
    {
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
        
        float min_distance = std::numeric_limits<float>::max();

        for (float w = -2.0; w < 2.0; w += 0.1) {
            float x_pred = x + v*delta_t*cos(theta+w*delta_t/2);
            float y_pred = y + v*delta_t*sin(theta+w*delta_t/2);
            float distance = sqrt(pow(x_pred - goal_pose.position.x, 2) + pow(y_pred - goal_pose.position.y, 2));
            
            float angle_to_goal = atan2(goal_pose.position.y - y_pred, goal_pose.position.x - x_pred);
            float angle_diff = angle_to_goal - (theta );
            angle_diff = atan2(sin(angle_diff), cos(angle_diff));
            while (angle_diff > M_PI) angle_diff -= 2 * M_PI;
            while (angle_diff < -M_PI) angle_diff += 2 * M_PI;
            if (distance < min_distance) {
                min_distance = distance;
                optimal_yaw_rate  = angle_diff / delta_t;;
            }

        }


        RCLCPP_INFO(this->get_logger(), "Optimal yaw rate: %f", optimal_yaw_rate);
        if (min_distance < 0.1) {
            geometry_msgs::msg::Twist cmd_vel_msg;
            cmd_vel_msg.linear.x = 0.0;
            cmd_vel_msg.angular.z = 0.0;
            cmd_vel_publisher->publish(cmd_vel_msg);
            
            RCLCPP_INFO(this->get_logger(), "Goal reached");
            return;
        }
        
        geometry_msgs::msg::Twist cmd_vel_msg;
        cmd_vel_msg.linear.x = v;
        cmd_vel_msg.angular.z = optimal_yaw_rate;
        cmd_vel_publisher->publish(cmd_vel_msg);
        RCLCPP_INFO(this->get_logger(), "Published cmd_vel: linear.x=%f, angular.z=%f", cmd_vel_msg.linear.x, cmd_vel_msg.angular.z);
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