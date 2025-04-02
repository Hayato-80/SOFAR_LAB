// this node has its class fully defined in the cpp file
// for larger nodes, the class can be classically split in header / source


// include any thing required - do not forget to use the .hpp extension for ROS 2 files
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <turtlesim/msg/pose.hpp>
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/static_transform_broadcaster.h"
// #include <my_msgs/msg/my_output_msg.hpp>
#include <algorithm>

using namespace std::chrono_literals;

namespace my_pkg
{

class MyNode : public rclcpp::Node
{
public:
    MyNode(rclcpp::NodeOptions options) : Node("my_node", options)
    {
        // init whatever is needed for your node
        
        // init subscribers
        my_subscription = this->create_subscription<turtlesim::msg::Pose>(
      		"/turtle1/pose", 10, std::bind(&MyNode::my_callback, this, std::placeholders::_1));
            
        // init publishers
        my_publisher = create_publisher<nav_msgs::msg::Odometry>("/odom", 10);   // topic + QoS
      
        // init timer - the function will be called with the given rate
        // publish_timer = create_wall_timer(100ms,    // rate
        //                                   [&](){callback_time();});
    }   
  
private:
    // declare any subscriber / publisher / timer
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr my_subscription;
    
    // MyInputMsg input_msg;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr my_publisher;
    
    void my_callback(const turtlesim::msg::Pose::SharedPtr msg)
    {
        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.stamp = this->get_clock()->now();
        odom_msg.header.frame_id = "odom";
        odom_msg.child_frame_id = "base_link";
        odom_msg.pose.pose.position.x = msg->x;
        odom_msg.pose.pose.position.y = msg->y;
        odom_msg.pose.pose.position.z = 0.0;
        
        tf2::Quaternion q;
        q.setRPY(0, 0, msg->theta);
        odom_msg.pose.pose.orientation.x = q.x();
        odom_msg.pose.pose.orientation.y = q.y();
        odom_msg.pose.pose.orientation.z = q.z();
        odom_msg.pose.pose.orientation.w = q.w();
        odom_msg.twist.twist.linear.x = msg->linear_velocity;
        odom_msg.twist.twist.linear.y = 0.0;
        odom_msg.twist.twist.linear.z = 0.0;
        odom_msg.twist.twist.angular.x = 0.0;
        odom_msg.twist.twist.angular.y = 0.0;
        odom_msg.twist.twist.angular.z = msg->angular_velocity;

        my_publisher->publish(odom_msg);
    }
};

}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  rclcpp::spin(std::make_shared<my_pkg::MyNode>(options));
  rclcpp::shutdown();
  return 0;
}
