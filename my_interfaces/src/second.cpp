#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "sensor_msgs/msg/range.hpp"
#include <cmath>

class RangeToOrigin : public rclcpp::Node
{
public:
  RangeToOrigin() : Node("range_to_origin")
  {
    // Declare the parameter with a default value
    this->declare_parameter("field_of_view", 90.0);

    // Get the parameter value
    double field_of_view;
    this->get_parameter("field_of_view", field_of_view);
    
    RCLCPP_INFO(this->get_logger(), "Field of View: %f", field_of_view);
    subscription_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/amcl_pose", 10,
      std::bind(&RangeToOrigin::pose_callback, this, std::placeholders::_1));

    publisher_ = this->create_publisher<sensor_msgs::msg::Range>("range_to_origin", 10);
  }
private:
  void pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;
    double distance = std::sqrt(x * x + y * y);

    RCLCPP_INFO(this->get_logger(), "Distance to origin: %.2f", distance);

    sensor_msgs::msg::Range range_msg;
    range_msg.header.stamp = this->get_clock()->now();
    range_msg.header.frame_id = "base_link";
    range_msg.radiation_type = sensor_msgs::msg::Range::ULTRASOUND;  // or INFRARED
    range_msg.field_of_view = 0.5;
    range_msg.min_range = 0.0;
    range_msg.max_range = 50.0;
    range_msg.range = static_cast<float>(distance);

    publisher_->publish(range_msg);
  }

  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RangeToOrigin>());
  rclcpp::shutdown();
  return 0;
}
