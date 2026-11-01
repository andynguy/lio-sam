#pragma once


#include <deque>
#include <mutex>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>

constexpr int MAX_BUFFER_SIZE = 200;

// Takes in gyroscope readings
// TODO(AN): create imu node reader that takes in gyroscope scans
// use gyroscope scans to deskew point cloud

// NOW WE NEED A WAY TO VERIFY THAT WE ARE DESKEWING POINT CLOUD, 
// gotta think of a way to test before and after to see if deskewing worked
class CloudProcessor : public rclcpp::Node
{
public:
struct Quaternion
{
    float x, y, z, w;
};
struct Position
{
    float x, y, z;
};
struct Velocity
{
    float x, y, z;
};
struct KeyFrame
{
    // From body frame to world frame
   Quaternion orientation;
   Position position;
   Velocity velocity;
   double time;
};

// TODO(an): CREATE LIDAR FRAME LOGIC ON PAGE 3/8
    CloudProcessor();
    void timer_callback();
    void processCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void storeImu(const sensor_msgs::msg::Imu::SharedPtr msg);
    void createRangeImage();
    void preintegrateImu(const rclcpp::Time& start, const rclcpp::Time& end);

private:
    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

    std::mutex imu_mutex_;
    std::deque<sensor_msgs::msg::Imu::SharedPtr> imu_buffer_;

    std::vector<KeyFrame> keyframes;

    //TODO(AN): turn these into eigen matrices 
    geometry_msgs::msg::Quaternion delta_q;
    geometry_msgs::msg::Vector3 delta_w;
    geometry_msgs::msg::Vector3 delta_a;

    
    
};
