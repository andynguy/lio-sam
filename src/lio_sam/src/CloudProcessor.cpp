#include "lio_sam/CloudProcessor.h"

using namespace std::chrono_literals;

CloudProcessor::CloudProcessor() : Node("cloud_processor")
{
    publisher = this->create_publisher<std_msgs::msg::String>("topic", 10);
    // Timer to trigger periodic publishing
    std::cout << "should start triggering timer..." << std::endl;
    timer = this->create_wall_timer(
             500ms, std::bind(&CloudProcessor::timer_callback, this));
    
    cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("/kitti/velo", 10,
           std::bind(&CloudProcessor::processCloud, this, std::placeholders::_1));
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>("/kitti/oxts/imu", 10,
           std::bind(&CloudProcessor::storeImu, this, std::placeholders::_1));
}

void CloudProcessor::timer_callback()
{
    auto message = std_msgs::msg::String();
    message.data = "Hello world!";
    publisher->publish(message);
}

void CloudProcessor::processCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Got point cloud: %d x %d points",
                msg->width, msg->height);
    
}

void CloudProcessor::storeImu(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(imu_mutex_);
    imu_buffer_.push_back(msg);

    if(imu_buffer_.size() > MAX_BUFFER_SIZE)
        imu_buffer_.pop_front();

    RCLCPP_INFO(this->get_logger(), "Buffer Size: %d", (int)imu_buffer_.size());

}

void CloudProcessor::createRangeImage() {}
// TODO(AN): DO THIS FIRSTTTTTTTTTTTTTTt!!!
// this will create the range image used for feature extraction finding edge and planes

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


    

    
    // add up R, p, v

    //

}
