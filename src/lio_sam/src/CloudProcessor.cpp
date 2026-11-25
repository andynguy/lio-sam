#include "lio_sam/CloudProcessor.h"

#include <cmath>

#include <tf2/exceptions.h>
#include "tf2_eigen/tf2_eigen.hpp"

namespace
{
constexpr char kImuFrame[] = "imu_link";
constexpr char kLidarFrame[] = "velo_link";
}  // namespace

CloudProcessor::CloudProcessor() : Node("cloud_processor")
{
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Raw cloud after the PCL round trip (step 2), for comparing against the deskewed cloud.
    raw_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/deskew/cloud_raw", 5);

    // Reliable rather than SensorDataQoS (best effort): a 2.7 MB cloud is split into many
    // network fragments, and with best effort losing one fragment drops the whole scan.
    // The IMU queue holds 10 s at 100 Hz, so a slow scan callback never drops IMU samples.
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>("/kitti/velo", rclcpp::QoS(5),
           std::bind(&CloudProcessor::processCloud, this, std::placeholders::_1));
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>("/kitti/oxts/imu", rclcpp::QoS(1000),
           std::bind(&CloudProcessor::storeImu, this, std::placeholders::_1));
}

bool CloudProcessor::loadExtrinsic()
{
    if (have_extrinsic_) return true;

    geometry_msgs::msg::TransformStamped tf_msg;
    try
    {
        // lookupTransform(target, source) returns the transform that maps points from the
        // source frame into the target frame, so this gives p_lidar = T_LI * p_imu.
        tf_msg = tf_buffer_->lookupTransform(kLidarFrame, kImuFrame, tf2::TimePointZero);
    }
    catch (const tf2::TransformException& e)
    {
        RCLCPP_WARN(get_logger(), "No %s -> %s transform yet, skipping this scan: %s",
                    kImuFrame, kLidarFrame, e.what());
        return false;
    }

    T_LI_ = tf2::transformToEigen(tf_msg);
    have_extrinsic_ = true;

    const Eigen::AngleAxisd aa(T_LI_.rotation());
    const Eigen::Vector3d& t = T_LI_.translation();
    RCLCPP_INFO(get_logger(),
                "Extrinsic %s -> %s: rotation %.3f deg about (%.3f, %.3f, %.3f), "
                "translation (%.3f, %.3f, %.3f) m",
                kImuFrame, kLidarFrame, aa.angle() * 180.0 / M_PI,
                aa.axis().x(), aa.axis().y(), aa.axis().z(), t.x(), t.y(), t.z());
    return true;
}

bool CloudProcessor::cacheCloud(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg)
{
    cloud_msg_queue_.push_back(cloud_msg);
    if (cloud_msg_queue_.size() <= 2) return false;
    current_cloud_msg_ = cloud_msg_queue_.front();
    pcl::moveFromROSMsg(*current_cloud_msg_, lidar_cloud_);
    current_cloud_header_ = current_cloud_msg_->header;
    cloud_msg_queue_.pop_front();
    if (lidar_cloud_.points.empty())
    {
        RCLCPP_WARN(get_logger(), "WARNING: Dropped lidar cloud scan because of empty cloud!");
        return false;
    }
    double time_start = rclcpp::Time(current_cloud_header_.stamp).seconds();
    double time_end = time_start + lidar_cloud_.points.back().time;
    if (!imuCovers(time_start, time_end))
    {    
        RCLCPP_WARN(get_logger(), "WARNING: Dropped lidar cloud scan because imu does not cover full scan!");
        return false;
    }
    return true;
}

bool CloudProcessor::imuCovers(double t0, double t1)
{
    std::lock_guard<std::mutex> lock(imu_mutex_);
    return !imu_buffer_.empty() &&
           rclcpp::Time(imu_buffer_.front()->header.stamp).seconds() <= t0 &&
           rclcpp::Time(imu_buffer_.back()->header.stamp).seconds() >= t1;
}

void CloudProcessor::processCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    if (!loadExtrinsic()) return;

    // Caches cloud to main lidar_cloud_ and publishes to topic. 
    // If IMU does not cover the scan, then drop the scan.
    if (!cacheCloud(msg))
        return;
    // convert back
    sensor_msgs::msg::PointCloud2 ros_out_msg;
    pcl::toROSMsg(lidar_cloud_, ros_out_msg);
    ros_out_msg.header = current_cloud_header_;
    raw_pub_->publish(ros_out_msg);
    RCLCPP_DEBUG(get_logger(), "Got point cloud: %u points", lidar_cloud_->width * lidar_cloud_->height);
}

void CloudProcessor::storeImu(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(imu_mutex_);
    if (!imu_buffer_.empty() && rclcpp::Time(msg->header.stamp) < rclcpp::Time(imu_buffer_.back()->header.stamp))
    {
        RCLCPP_WARN(get_logger(), "Out of time order imu measurement at %f", 
                                  rclcpp::Time(imu_buffer_.back()->header.stamp).seconds());
        return;
    }
    imu_buffer_.push_back(msg);

    if (imu_buffer_.size() > MAX_BUFFER_SIZE)
        imu_buffer_.pop_front();
}

void CloudProcessor::createRangeImage() {}
// Step 10: project the deskewed cloud onto the 64 x 2000 range image.

// use for after new lidar scan comes in
void CloudProcessor::preintegrateImu(const rclcpp::Time& start, const rclcpp::Time& end)
{
    // UNFINISHED, maybe do this for two frames
    //LOCK mutex copy imu vector here

    std::deque<sensor_msgs::msg::Imu::SharedPtr> bufferCopy;
    {
        std::lock_guard<std::mutex> lock(imu_mutex_);
        for (sensor_msgs::msg::Imu::SharedPtr imuReading : imu_buffer_)
        {
            rclcpp::Time imuTime = rclcpp::Time(imuReading->header.stamp);
            if (imuTime < start)
                continue;
            if (imuTime > end)
                break;
            bufferCopy.push_back(imuReading);
        }
    }
}
