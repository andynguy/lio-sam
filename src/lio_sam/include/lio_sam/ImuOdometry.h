#pragma once

#include <vector>

#include <Eigen/Dense>

#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include "lio_sam/MathUtils.h"

class ImuOdometry : public rclcpp::Node
{
public:
    ImuOdometry();

private:
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
    void groundTruthCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void initialize(const nav_msgs::msg::Odometry& gt, const rclcpp::Time& stamp);
    void publishOdometry(const rclcpp::Time& stamp, const Eigen::Vector3d& gyro);
    void publishPaths();
    void logImuDrift(const rclcpp::Time& stamp);

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr gt_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr imu_path_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr gt_path_pub_;
    rclcpp::TimerBase::SharedPtr path_timer_;

    utils::NavState state_;
    utils::ImuBias bias_;
    Eigen::Vector3d gravity_;

    bool initialized_;
    bool have_gt_;
    rclcpp::Time start_time_;
    rclcpp::Time last_imu_time_;
    nav_msgs::msg::Odometry latest_gt_;

    nav_msgs::msg::Path imu_path_;
    nav_msgs::msg::Path gt_path_;

    std::vector<double> imu_drift_error_checkpoints_;
    std::size_t imu_drift_next_checkpoint_{0};
};
