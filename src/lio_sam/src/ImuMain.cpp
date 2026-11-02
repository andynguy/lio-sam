#include "rclcpp/rclcpp.hpp"
#include "lio_sam/ImuOdometry.h"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    std::cout << "Spinning imu odometry processing...\n";
    rclcpp::spin(std::make_shared<ImuOdometry>());
    rclcpp::shutdown();
    return 0;
}
