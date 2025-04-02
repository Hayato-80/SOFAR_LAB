// this node has its class fully defined in the cpp file
// for larger nodes, the class can be classically split in header / source


// include any thing required - do not forget to use the .hpp extension for ROS 2 files
#include <rclcpp/rclcpp.hpp>
#include <my_msgs/msg/my_input_msg.hpp>
#include <my_msgs/msg/my_output_msg.hpp>
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
        my_subscription = create_subscription<my_msgs::msg::MyInputMsg>(
            "my_input_topic",    // which topic
            10,         // QoS            
            [this](my_msgs::msg::MyInputMsg::UniquePtr msg)    // callback are perfect for lambdas
            {
                input_msg = *msg;
            });
            
        // init publishers
        my_publisher = create_publisher<my_msgs::msg::MyOutputMsg>("my_output_topic", 10);   // topic + QoS
      
        // init timer - the function will be called with the given rate
        publish_timer = create_wall_timer(100ms,    // rate
                                          [&](){callback_time();});
    }   
  
private:
    // declare any subscriber / publisher / timer
    rclcpp::Subscription<my_msgs::msg::MyInputMsg>::SharedPtr my_subscription;
    
    MyInputMsg input_msg;

    rclcpp::Publisher<my_msgs::msg::MyOutputMsg>::SharedPtr my_publisher;
    
    rclcpp::TimerBase::SharedPtr publish_timer;
    
    void callback_time()
    {
        my_msgs::msg::MyOutputMsg msg;
        my_publisher->publish(msg);
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
