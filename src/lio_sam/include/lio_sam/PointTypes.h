#pragma once
#define PCL_NO_PRECOMPILE
#include <cstdint>
#include <pcl/point_types.h>

struct PointXYZIRT
{
    PCL_ADD_POINT4D;          // x, y, z + 4 bytes padding (16-byte aligned for SIMD)
    float intensity;
    std::uint16_t ring;       // laser index 0..63; must match the bag's UINT16
    float time;               // seconds since sweep start (header.stamp), 0..~0.1035
    PCL_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

// (C++ type, struct member, message field name): the field name must match the bag exactly,
// or pcl::fromROSMsg leaves that member unset and prints "Failed to find match for field".
POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIRT,
    (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)
    (std::uint16_t, ring, ring)(float, time, time))
