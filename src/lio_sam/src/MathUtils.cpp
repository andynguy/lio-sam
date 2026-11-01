#include "lio_sam/MathUtils.h"

namespace utils
{

Eigen::Matrix3d skew(const Eigen::Vector3d& v)
{
    Eigen::Matrix3d skewedMat;
    skewedMat << 0, -v.z(), v.y(),
                 v.z(), 0, -v.x(),
                 -v.y(), v.x(), 0;
    return skewedMat;
}

Eigen::Matrix3d expSO3(const Eigen::Vector3d& phi)
{
    double theta = phi.norm();
    if (theta < 1e-8) return Eigen::Matrix3d::Identity() + skew(phi);
    Eigen::Matrix3d w = skew(phi / theta);
    return Eigen::Matrix3d::Identity() + sin(theta)*w + (1 - cos(theta))*w*w;
}

/* 
state: current robot state, where R is body->world, written in world coordinates, position p and velocity v is in world coords
acc: accelerometer reading in the body frame (m/s^2)
gyro: rotational velocity reading (w) in body frame (rad/s)
dt: seconds between last sample, from imu timestamps (s)
bias: acceleration (m/s^2) and gyroscope (rad/s) imu biases
gravity: acceleration of gravity in world frame, default = (0, 0, -9.81)
*/

void integrate(NavState& state,
               const Eigen::Vector3d& acc,
               const Eigen::Vector3d& gyro,
               double dt,
               const ImuBias& bias,
               const Eigen::Vector3d& gravity)
{
    Eigen::Vector3d vel_new = state.v + gravity*dt + state.R*(acc - bias.ba)*dt;
    Eigen::Vector3d pos_new = state.p + state.v*dt + 0.5*gravity*dt*dt + 0.5*state.R*(acc-bias.ba)*dt*dt;
    Eigen::Matrix3d R_new = state.R*expSO3((gyro-bias.bg)*dt);

    state.R = R_new;
    state.p = pos_new;
    state.v = vel_new;
}

}