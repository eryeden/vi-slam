#pragma once

#include "type_defines.hpp"

namespace vislam::data
{

class landmark
{

public:
    landmark(uint64_t id,
            const std::unordered_set<uint64_t > & observing_feature_id,
            const eigen_allocated_unordered_map<uint64_t , Vec2_t> &observing_feature_point_in_device,
            const Vec3_t & camera_position,
            const Quat_t & camera_attitude,
            const Mat33_t & camera_intrinsic_parameter);
    explicit landmark(uint64_t id);
    landmark();

    uint64_t id;
    std::unordered_set<uint64_t > observingFeatureId;
    eigen_allocated_unordered_map<uint64_t , Vec2_t> observingFeaturePointInDevice;

    Vec3_t cameraPosition;
    Quat_t cameraAttitude;
    Mat33_t cameraIntrinsicParameter;


private:


};

} // namespace vislam