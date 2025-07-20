#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/int64.hpp"
#include "example_interfaces/srv/set_bool.hpp"

using namespace std::placeholders;
 
class NumberCounterNode : public rclcpp::Node
{
public:
   NumberCounterNode() : Node("number_counter"), counter_(0) // constrctor initilazation is recommended
   {
    subscriber_ = this->create_subscription<example_interfaces::msg::Int64>(
        "number",10,
        std::bind(&NumberCounterNode::callbackNumber, this,_1));
    publisher_ = this->create_publisher<example_interfaces::msg::Int64>("number_count",10);
    
    server_ = this->create_service<example_interfaces::srv::SetBool>("reset_counter",
               std::bind(&NumberCounterNode::callbackResetNumber,this,_1,_2));
    
   }
 
private:
   void callbackNumber(const example_interfaces::msg::Int64::SharedPtr msg) //means that the msg object is passed by value as a constant
   {
  
     counter_ += msg->data; //data accumaulation eg:If counter_ was 10 and msg->data is 5, after this line counter_ will be 15
                              //If the next message has msg->data as -3, counter_ will become 12

      RCLCPP_INFO(this->get_logger(), "Number of messages recived = %ld",counter_);

      auto msg2 = example_interfaces::msg::Int64();
      msg2.data = counter_;
      publisher_->publish(msg2);
      
        
   }
   void callbackResetNumber(const example_interfaces::srv::SetBool::Request::SharedPtr request,
                            const example_interfaces::srv::SetBool::Response::SharedPtr response)
   {
      if (request->data == true)
      {
         counter_ = 0;
         response->message = "conter SET to zero";
         response->success = true;
      }
      else
      {
         response->message = "conter NOT SET to zero";
         response->success = false;
      }

   }

   rclcpp::Subscription<example_interfaces::msg::Int64>::SharedPtr subscriber_;
   rclcpp::Publisher<example_interfaces::msg::Int64>::SharedPtr publisher_;
   std::int64_t counter_;
   rclcpp::Service<example_interfaces::srv::SetBool>::SharedPtr server_;
};
 
int main(int argc, char **argv)
{
   rclcpp::init(argc, argv);
   auto node = std::make_shared<NumberCounterNode>(); 
   rclcpp::spin(node);
   rclcpp::shutdown();
   return 0;
}
