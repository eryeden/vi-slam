#pragma once

#include "type_defines.hpp"
#include "camera.hpp"

namespace vislam::data
{


struct frame
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    frame(uint64_t id,
          const std::unordered_set<uint64_t > & observing_feature_id,
              const eigen_allocated_unordered_map<uint64_t , Vec2_t> &observing_feature_point_in_device,
          const Vec3_t & camera_position,
              const Quat_t & camera_attitude,
              const Mat33_t & camera_intrinsic_parameter);
    frame(uint64_t id,
          const std::unordered_set<uint64_t > & observing_feature_id,
          const eigen_allocated_unordered_map<uint64_t , Vec2_t> &observing_feature_point_in_device,
          const Vec3_t & camera_position,
          const Quat_t & camera_attitude);
    explicit frame(uint64_t id);
    frame();



    uint64_t id;
    std::unordered_set<uint64_t > observingFeatureId;
    eigen_allocated_unordered_map<uint64_t , Vec2_t> observingFeaturePointInDevice;

    Vec3_t cameraPosition;
    Quat_t cameraAttitude;

    camera cameraParameter;
private:
};

} // namespace vislam
