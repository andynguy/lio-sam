#include "lio_sam/ImuOdometry.h"

using namespace std::chrono_literals;

namespace
{
Eigen::Vector3d toEigen(const geometry_msgs::msg::Point& p)
{
    return Eigen::Vector3d(p.x, p.y, p.z);
}

geometry_msgs::msg::Point toPoint(const Eigen::Vector3d& v)
{
    geometry_msgs::msg::Point p;
    p.x = v.x();
    p.y = v.y();
    p.z = v.z();
    return p;
}

geometry_msgs::msg::Quaternion toQuaternion(const Eigen::Matrix3d& R)
{
    // Normalize because R drifts slightly from a pure rotation after many integration steps.
    const Eigen::Quaterniond q = Eigen::Quaterniond(R).normalized();
    geometry_msgs::msg::Quaternion out;
    out.w = q.w();
    out.x = q.x();
    out.y = q.y();
    out.z = q.z();
    return out;
}
}  // namespace

ImuOdometry::ImuOdometry() : Node("imu_odometry"), 
                             state_{Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()},
                             bias_{Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()},
                             gravity_(0, 0, -9.81),
                             initialized_(false),
                             have_gt_(false)
{
    imu_path_.header.frame_id = gt_path_.header.frame_id = "map";
    path_timer_ = this->create_wall_timer(
             500ms, std::bind(&ImuOdometry::publishPaths, this));
    imu_path_pub_ = create_publisher<nav_msgs::msg::Path>("/imu_path", 10);
    gt_path_pub_ = create_publisher<nav_msgs::msg::Path>("/gt_path", 10);
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/imu_odom", 10);

    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>("/kitti/oxts/imu", rclcpp::QoS(1000),
                    std::bind(&ImuOdometry::imuCallback, this, std::placeholders::_1));
    gt_sub_ = create_subscription<nav_msgs::msg::Odometry>("/kitti/ground_truth/odom", rclcpp::QoS(1000),
                    std::bind(&ImuOdometry::groundTruthCallback, this, std::placeholders::_1));
    imu_drift_error_checkpoints_ = {0, 10, 20, 30, 40, 50};
}

void ImuOdometry::initialize(const nav_msgs::msg::Odometry& gt, const rclcpp::Time& stamp)
{
    const auto& o = gt.pose.pose.orientation;
    const auto& pos = gt.pose.pose.position;
    const auto& vel = gt.twist.twist.linear;

    const Eigen::Quaterniond q(o.w, o.x, o.y, o.z);
    state_.R = q.normalized().toRotationMatrix();
    state_.p = Eigen::Vector3d(pos.x, pos.y, pos.z);
    const Eigen::Vector3d v_body(vel.x, vel.y, vel.z);
    state_.v = state_.R * v_body; // world coordinates
    start_time_ = stamp;
    last_imu_time_ = stamp;
    initialized_ = true;

    RCLCPP_INFO(get_logger(),
                "Initialized from ground truth: p = (%.2f, %.2f, %.2f)  v_W = (%.2f, %.2f, %.2f)  |v| = %.2f m/s",
                state_.p.x(), state_.p.y(), state_.p.z(),
                state_.v.x(), state_.v.y(), state_.v.z(), state_.v.norm());
}

void ImuOdometry::publishPaths()
{
    //publishes imu_paths_ and gt_paths_
    imu_path_.header.stamp = gt_path_.header.stamp = now();
    imu_path_pub_->publish(imu_path_);
    gt_path_pub_->publish(gt_path_);
}

void ImuOdometry::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{

    //integrates imu data published from kitti to robot state
    if (!initialized_)
    {
        if (have_gt_) initialize(latest_gt_, msg->header.stamp);
        return;
    }
    double dt = (rclcpp::Time(msg->header.stamp) - last_imu_time_).seconds();
    if (dt <= 0)
    {
        RCLCPP_WARN(get_logger(), "Imu messages are out of order, dropping message.");
        return;
    }
    if (dt > 0.2)
    {
        RCLCPP_WARN(get_logger(), "Large jump in imu time stamps, skipping integration and resyncing.");
        last_imu_time_ = rclcpp::Time(msg->header.stamp);
        return;
    }
    last_imu_time_ = msg->header.stamp;
    Eigen::Vector3d acc{msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z};
    Eigen::Vector3d gyro{msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z};

    utils::integrate(state_,
                     acc,
                     gyro,
                     dt,
                     bias_,
                     gravity_);

    // Append the new state to the dead-reckoning trail. A Path holds PoseStamped elements,
    // and each one carries its own header (the Path's header is set in publishPaths).
    geometry_msgs::msg::PoseStamped pose_new;
    pose_new.header.stamp = msg->header.stamp;
    pose_new.header.frame_id = "map";
    pose_new.pose.position = toPoint(state_.p);
    pose_new.pose.orientation = toQuaternion(state_.R);
    imu_path_.poses.push_back(pose_new);
    publishOdometry(msg->header.stamp, gyro);
    logImuDrift(msg->header.stamp);
}

void ImuOdometry::groundTruthCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    have_gt_ = true;
    latest_gt_ = *msg;
    // msg->pose is a PoseWithCovariance; the Path wants a PoseStamped, so take the inner
    // Pose (msg->pose.pose) and pair it with the message's header (stamp + "map" frame).
    geometry_msgs::msg::PoseStamped pose_gt;
    pose_gt.header = msg->header;
    pose_gt.pose = msg->pose.pose;
    gt_path_.poses.push_back(pose_gt);
}

void ImuOdometry::publishOdometry(const rclcpp::Time& stamp, const Eigen::Vector3d& gyro)
{
    nav_msgs::msg::Odometry odomMsg;
    odomMsg.header.stamp = stamp;
    odomMsg.header.frame_id = "map";
    odomMsg.child_frame_id = "imu_link";
    odomMsg.pose.pose.position = toPoint(state_.p);
    odomMsg.pose.pose.orientation = toQuaternion(state_.R);
    Eigen::Vector3d v_body = state_.R.transpose() * state_.v;
    odomMsg.twist.twist.linear.x = v_body.x();
    odomMsg.twist.twist.linear.y = v_body.y();
    odomMsg.twist.twist.linear.z = v_body.z();
    Eigen::Vector3d w = gyro - bias_.bg;
    odomMsg.twist.twist.angular.x = w.x();
    odomMsg.twist.twist.angular.y = w.y();
    odomMsg.twist.twist.angular.z = w.z();
    odom_pub_->publish(odomMsg);
}
    
void ImuOdometry::logImuDrift(const rclcpp::Time& stamp)
{
    if (imu_drift_next_checkpoint_ >= imu_drift_error_checkpoints_.size()) return;
    double elapsedTime = (stamp - start_time_).seconds();
    if (elapsedTime >= imu_drift_error_checkpoints_[imu_drift_next_checkpoint_])
    {
        double error = (toEigen(latest_gt_.pose.pose.position) - state_.p).norm();
        RCLCPP_INFO(get_logger(),
                    "Imu Drift Error (m): %.2f at time %.2f", error, elapsedTime);
        imu_drift_next_checkpoint_++;
    }
}
                             