#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include <cmath>

class RangeToOrigin : public rclcpp::Node
{
public:
  RangeToOrigin() : Node("range_to_origin")
  {
    subscription_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/amcl_pose", 10,
      std::bind(&RangeToOrigin::pose_callback, this, std::placeholders::_1));
  }

private:
  void pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;
    double distance = std::sqrt(x * x + y * y);

    RCLCPP_INFO(this->get_logger(), "Distance to origin: %.2f", distance);
  }

  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RangeToOrigin>());
  rclcpp::shutdown();
  return 0;
}
