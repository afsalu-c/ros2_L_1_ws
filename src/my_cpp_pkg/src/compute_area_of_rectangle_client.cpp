#include "rclcpp/rclcpp.hpp"
#include "my_robot_interfaces/srv/compute_rectangle_area.hpp"
#include <iostream>

using namespace std::chrono_literals;
using namespace std::placeholders;
 
// class ComputeRectangleAreaClient : public rclcpp::Node 
// {
// public:
//    ComputeRectangleAreaClient() : Node("compute_rectangle_area_client") 
//    {
//     rectangle_area_client_ = this->create_client<my_robot_interfaces::srv::ComputeRectangleArea>("rectangle_area");
//     // timer_ = this->create_wall_timer(1s,std::bind(&ComputeRectangleAreaClient::compute_rectangle_area_client,this));
//    }
//     void compute_rectangle_area_client()
//      {
//         while (!rectangle_area_client_->wait_for_service(1s))
//             {
//             RCLCPP_WARN(this->get_logger(), "Waiting for compute rectangle area service server ......");
//             }
//         float l, w;
//         std::cout << "Enter the length and width of the rectangle one by one as float" << std::endl;
//         std::cin >> l >> w;
//         auto request = std::make_shared<my_robot_interfaces::srv::ComputeRectangleArea::Request>();
//         request->length = l;
//         request->width = w;
//         rectangle_area_client_->async_send_request(request,std::bind(&ComputeRectangleAreaClient::callback_compute_rectangle_area_client,this, _1));
//      }
 
// private:


//    void callback_compute_rectangle_area_client(rclcpp::Client<my_robot_interfaces::srv::ComputeRectangleArea>::SharedFuture future)
//    {
//     auto response = future.get();
//     RCLCPP_INFO(this->get_logger(),"Area = %f", (float)response->area);

//    }
//    rclcpp::Client<my_robot_interfaces::srv::ComputeRectangleArea>::SharedPtr rectangle_area_client_;
// //    rclcpp::TimerBase::SharedPtr timer_;

// };
 
// int main(int argc, char **argv)
// {
//    rclcpp::init(argc, argv);
//    auto node = std::make_shared<ComputeRectangleAreaClient>(); 
//    node->compute_rectangle_area_client();
//    rclcpp::spin(node);
//    rclcpp::shutdown();
//    return 0;
// }





//------------GPT CODE -----------

class ComputeRectangleAreaClient : public rclcpp::Node 
{
public:
    ComputeRectangleAreaClient() : Node("compute_rectangle_area_client") 
    {
        rectangle_area_client_ = this->create_client<my_robot_interfaces::srv::ComputeRectangleArea>("rectangle_area");
        RCLCPP_INFO(this->get_logger(), "compute rectangle area service client has been started");

        // Wait for service once at the start
        while (!rectangle_area_client_->wait_for_service(1s)) {
            RCLCPP_WARN(this->get_logger(), "Waiting for compute rectangle area service server ......");
        }

        // Start a separate thread to avoid blocking the executor
        std::thread([this]() {
            while (rclcpp::ok()) {
                float l, w;
                // std::cout << "Enter the length : ";
                // std::cin >> l ;
                // std::cout << "Enter the width : ";
                // std::cin >> w ;
                // std::cout << std::endl;
                std::cout << "Enter the length and width of the rectangle one by one as float: ";
                std::cin >> l >> w;

                auto request = std::make_shared<my_robot_interfaces::srv::ComputeRectangleArea::Request>();
                request->length = l;
                request->width = w;

                auto response_callback = std::bind(
                    &ComputeRectangleAreaClient::callback_callback_compute_rectangle_area_client,
                    this, _1
                );

                rectangle_area_client_->async_send_request(request, response_callback);
            }
        }).detach();  // Detach the thread so it runs independently
    }

private:
    void callback_callback_compute_rectangle_area_client(
        rclcpp::Client<my_robot_interfaces::srv::ComputeRectangleArea>::SharedFuture future)
    {
        auto response = future.get();
        RCLCPP_INFO(this->get_logger(), "Area = %f", static_cast<float>(response->area));
    }

    rclcpp::Client<my_robot_interfaces::srv::ComputeRectangleArea>::SharedPtr rectangle_area_client_;
};

int main(int argc, char **argv)
{
   rclcpp::init(argc, argv);
   auto node = std::make_shared<ComputeRectangleAreaClient>(); 
   rclcpp::spin(node);
   rclcpp::shutdown();
   return 0;
}