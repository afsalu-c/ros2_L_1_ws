#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/string.hpp"

using namespace std::placeholders;
 
class SmartphoneNode : public rclcpp::Node 
{
public:
   SmartphoneNode() : Node("smartpone") 
   {
        subsriber_ = this->create_subscription<example_interfaces::msg::String>(
            "robot_news", 10,
            std::bind(&SmartphoneNode::callbackRobotNews, this, _1)); //for 2 argument std::placeholders::_1, "std::placeholders::_2 are needed"
            RCLCPP_INFO(this->get_logger(), "Smartphone has been started.");
   }
 
private:
   void callbackRobotNews(const example_interfaces::msg::String::SharedPtr msg) // callback followed by topic namee 'RobotNews'
   {
        RCLCPP_INFO(this->get_logger(),"%s", msg->data.c_str()); //cast to string 
   }
   rclcpp::Subscription<example_interfaces::msg::String>::SharedPtr subsriber_;
};
 
int main(int argc, char **argv)
{
   rclcpp::init(argc, argv);
   auto node = std::make_shared<SmartphoneNode>(); 
   rclcpp::spin(node);
   rclcpp::shutdown();
   return 0;
}
