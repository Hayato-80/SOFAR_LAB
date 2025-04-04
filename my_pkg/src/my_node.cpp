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
// #include <my_msgs/msg/my_output_msg.hpp>


using namespace std::chrono_literals;

namespace my_pkg
{

class MyNode : public rclcpp::Node
{
public:
    MyNode(rclcpp::NodeOptions options) : Node("my_node", options)
    {
        // init subscribers
        my_subscription = this->create_subscription<turtlesim::msg::Pose>(
      		"/turtle1/pose", 10, std::bind(&MyNode::my_callback, this, std::placeholders::_1));
            
        // init publishers
        my_publisher = create_publisher<nav_msgs::msg::Odometry>("/odom", 10);   // topic + QoS
        tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(this);
        static_tf_broadcaster = std::make_unique<tf2_ros::StaticTransformBroadcaster>(this);

        // init timer - the function will be called with the given rate
        // publish_timer = create_wall_timer(100ms,    // rate
        //                                   [&](){callback_time();});
    }   
  
private:
    // declare any subscriber / publisher / timer
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr my_subscription;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
    std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster;
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
        geometry_msgs::msg::TransformStamped t;

        // Read message content and assign it to
        // corresponding tf variables
        t.header.stamp = this->get_clock()->now();
        t.header.frame_id = "odom";
        t.child_frame_id = "base_link";

        // Turtle only exists in 2D, thus we get x and y translation
        // coordinates from the message and set the z coordinate to 0
        t.transform.translation.x = msg->x;
        t.transform.translation.y = msg->y;
        t.transform.translation.z = 0.0;

        // For the same reason, turtle can only rotate around one axis
        // and this why we set rotation in x and y to 0 and obtain
        // rotation in z axis from the message
        // tf2::Quaternion q;
        // q.setRPY(0, 0, msg->theta);
        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        t.transform.rotation.w = q.w();

        // Send the transformation
        tf_broadcaster->sendTransform(t);

        geometry_msgs::msg::TransformStamped st;

        // Read message content and assign it to
        // corresponding tf variables
        st.header.stamp = this->get_clock()->now();
        st.header.frame_id = "map";
        st.child_frame_id = "odom";

        // Turtle only exists in 2D, thus we get x and y translation
        // coordinates from the message and set the z coordinate to 0
        st.transform.translation.x = -5.5;
        st.transform.translation.y = -5.5;
        st.transform.translation.z = 0.0;

        // For the same reason, turtle can only rotate around one axis
        // and this why we set rotation in x and y to 0 and obtain
        // rotation in z axis from the message
        // tf2::Quaternion q;
        // q.setRPY(0, 0, msg->theta);
        // st.transform.rotation.x = q.x();
        // st.transform.rotation.y = q.y();
        // st.transform.rotation.z = q.z();
        // st.transform.rotation.w = q.w();

        // Send the transformation
        static_tf_broadcaster->sendTransform(st);

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
