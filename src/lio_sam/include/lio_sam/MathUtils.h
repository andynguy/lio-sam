#pragma once

#include <Eigen/Dense>

namespace utils
{

struct NavState
{
    Eigen::Matrix3d R; // R_WB, body position in world coordinates
    Eigen::Vector3d p; // position in meters in world coordinates
    Eigen::Vector3d v; // velocity (m/s) in world coordinates
};

struct ImuBias
{
    Eigen::Vector3d ba;
    Eigen::Vector3d bg;
};

// Converts 3D vector into a skew symmetric 3x3 matrix
Eigen::Matrix3d skew(const Eigen::Vector3d& v);

// Skew-Symmetric Representation
// Uses Rodrigues' formula to have a exponential map to convert point 
// from Lie algebra to Lie group's manifold
// tangential space to curved manifold, so(3) -> SO(3)
Eigen::Matrix3d expSO3(const Eigen::Vector3d& phi);

// Advances robot state by one IMU sample
void integrate(NavState& state,
               const Eigen::Vector3d& acc,
               const Eigen::Vector3d& gyro,
               double dt,
               const ImuBias& bias,
               const Eigen::Vector3d& gravity);

}
