#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <unordered_map>
#include <cmath>
#include <vector>
#include <memory>
#include <my_interfaces/srv/get_turtle_pose.hpp>

using namespace std::chrono_literals;

namespace my_pkg
{

class Pose_Server : public rclcpp::Node
{
public:
    Pose_Server(rclcpp::NodeOptions options) : Node("get_turtle_pose", options)
    {
        // init whatever is needed for your node
        // init subscribers
        turtle_names_ = {"turtle2"};
        for(const auto& name: turtle_names_)
        {
            position_subscriptions_[name] = this->create_subscription<turtlesim::msg::Pose>(
                "/" + name + "/pose", 10,
                [this, name](const turtlesim::msg::Pose::SharedPtr msg) {
                    this->pose_callback(msg, name);
                });
        }

        turtle_pose_service_ = this->create_service<my_interfaces::srv::GetTurtlePose>(
            "get_turtle_pose", std::bind(&Pose_Server::get_turtle_poses, this, std::placeholders::_1, std::placeholders::_2));
        
    }
  
private:
    rclcpp::Service<my_interfaces::srv::GetTurtlePose>::SharedPtr turtle_pose_service_;
    std::unordered_map<std::string, rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr> position_subscriptions_;
    std::unordered_map<std::string, turtlesim::msg::Pose> turtle_poses_;
    std::vector<std::string> turtle_names_;
    
    void pose_callback(const turtlesim::msg::Pose::SharedPtr msg, const std::string& turtle_name)
    {
        turtle_poses_[turtle_name] = *msg;
    }

    void get_turtle_poses(std::shared_ptr<my_interfaces::srv::GetTurtlePose::Request> request,
        std::shared_ptr<my_interfaces::srv::GetTurtlePose::Response> response)
    {
        std::string turtle_name = request->turtle_name;
        if(turtle_poses_.find(turtle_name) != turtle_poses_.end())
        {

            const auto& pose = turtle_poses_[turtle_name];
            response->x = pose.x;
            response->y = pose.y;
            response->theta = pose.theta;
        }
    }
};

}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  rclcpp::spin(std::make_shared<my_pkg::Pose_Server>(options));
  rclcpp::shutdown();
  return 0;
}
