#include "rclcpp/rclcpp.hpp"
#include "lio_sam/CloudProcessor.h"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    std::cout << "hello, spinning cloud processor" << std::endl;
    rclcpp::spin(std::make_shared<CloudProcessor>());
    rclcpp::shutdown();
    return 0;
}