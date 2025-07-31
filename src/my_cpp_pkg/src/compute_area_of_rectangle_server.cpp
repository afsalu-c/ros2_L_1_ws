#include "rclcpp/rclcpp.hpp"
#include "my_robot_interfaces/srv/compute_rectangle_area.hpp"
using namespace std::placeholders;
 
class ComputeRectangleArea : public rclcpp::Node 
{
public:
   ComputeRectangleArea() : Node("compute_rectangle_area") 
   {
      rectangle_area_service_ = this->create_service<my_robot_interfaces::srv::ComputeRectangleArea>("rectangle_area",std::bind(&ComputeRectangleArea::callbackRectangleArea,this,_1,_2));
      RCLCPP_INFO(this->get_logger(),"Compute rectangle area node has been started");
   }
 
private:
   void callbackRectangleArea(const my_robot_interfaces::srv::ComputeRectangleArea::Request::SharedPtr request,
                                 const my_robot_interfaces::srv::ComputeRectangleArea::Response::SharedPtr response)
      {
         response->area = request->length * request->width;
         RCLCPP_INFO(this->get_logger(),"Area of rectangle with length:%f and width:%f is = %f", (float)request->length, (float)request->width, (float)response->area);
      } 
    rclcpp::Service<my_robot_interfaces::srv::ComputeRectangleArea>::SharedPtr rectangle_area_service_ ;
};
   
 
int main(int argc, char **argv)
{
   rclcpp::init(argc, argv);
   auto node = std::make_shared<ComputeRectangleArea>(); 
   rclcpp::spin(node);
   rclcpp::shutdown();
   return 0;
}
