#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <map>
#include <set>
#include <sophus/se3.hpp>
#include <sophus/so3.hpp>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace vslam {

// floating point type

typedef float real_t;

typedef uint64_t database_index_t;

// Eigen matrix types

template <size_t R, size_t C>
using MatRC_t = Eigen::Matrix<double, R, C>;

using Mat22_t = Eigen::Matrix2d;

using Mat33_t = Eigen::Matrix3d;

using Mat44_t = Eigen::Matrix4d;

using Mat55_t = MatRC_t<5, 5>;

using Mat66_t = MatRC_t<6, 6>;

using Mat77_t = MatRC_t<7, 7>;

using Mat34_t = MatRC_t<3, 4>;

using MatX_t = Eigen::MatrixXd;

// Eigen vector types

template <size_t R> using VecR_t = Eigen::Matrix<double, R, 1>;

using Vec2_t = Eigen::Vector2d;

using Vec3_t = Eigen::Vector3d;

using Vec4_t = Eigen::Vector4d;

using Vec5_t = VecR_t<5>;

using Vec6_t = VecR_t<6>;

using Vec7_t = VecR_t<7>;

using VecX_t = Eigen::VectorXd;

// Eigen Quaternion type

using Quat_t = Eigen::Quaterniond;

// Sophus
using Pose_t = Sophus::SE3d;
using Rot_t = Sophus::SO3d;

// STL with Eigen custom allocator
template <typename T>
using EigenAllocatedVector = std::vector<T, Eigen::aligned_allocator<T>>;

template <typename T, typename U>
using EigenAllocatedMap = std::
    map<T, U, std::less<T>, Eigen::aligned_allocator<std::pair<const T, U>>>;

template <typename T>
using EigenAllocatedSet =
    std::set<T, std::less<T>, Eigen::aligned_allocator<const T>>;

template <typename T, typename U>
using EigenAllocatedUnorderedMap =
    std::unordered_map<T,
                       U,
                       std::hash<T>,
                       std::equal_to<T>,
                       Eigen::aligned_allocator<std::pair<const T, U>>>;

template <typename T>
using EigenAllocatedUnorderedSet =
    std::unordered_set<T,
                       std::hash<T>,
                       std::equal_to<T>,
                       Eigen::aligned_allocator<const T>>;

using FeaturePositionDatabase =
    EigenAllocatedUnorderedMap<database_index_t, Vec2_t>;
using FeatureBearingDatabase =
    EigenAllocatedUnorderedMap<database_index_t, Vec3_t>;
using FeatureAgeDatabase = std::unordered_map<database_index_t, uint32_t>;

}  // namespace vslam