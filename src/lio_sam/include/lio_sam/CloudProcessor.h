#pragma once

// First, so PCL_NO_PRECOMPILE is defined before any other header pulls in PCL.
#include "lio_sam/PointTypes.h"

#include <deque>
#include <memory>
#include <mutex>

#include <Eigen/Geometry>

#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include "pcl_conversions/pcl_conversions.h"

constexpr int MAX_BUFFER_SIZE = 200;  // Step 4 replaces count-based pruning with time-based

// Phase 3 (image_projection): deskews each lidar sweep with the IMU, then builds the range image.
class CloudProcessor : public rclcpp::Node
{
public:
    CloudProcessor();

private:
    void processCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void storeImu(const sensor_msgs::msg::Imu::SharedPtr msg);
    bool imuCovers(double t0, double t1);
    bool loadExtrinsic();
    void createRangeImage();
    void preintegrateImu(const rclcpp::Time& start, const rclcpp::Time& end);
    bool cacheCloud(const sensor_msgs::msg::PointCloud2::SharedPtr cloud);

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr raw_pub_;  // /deskew/cloud_raw

    std::mutex imu_mutex_;
    std::deque<sensor_msgs::msg::Imu::SharedPtr> imu_buffer_;
    std::deque<sensor_msgs::msg::PointCloud2::SharedPtr> cloud_msg_queue_;
    pcl::PointCloud<PointXYZIRT>::Ptr lidar_cloud_;
    sensor_msgs::msg::PointCloud2::SharedPtr current_cloud_msg_;
    std_msgs::msg::Header current_cloud_header_;
    

    // IMU -> lidar extrinsic, read once from /tf_static.
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    bool have_extrinsic_{false};
    Eigen::Isometry3d T_LI_{Eigen::Isometry3d::Identity()};  // p_lidar = T_LI_ * p_imu


};
