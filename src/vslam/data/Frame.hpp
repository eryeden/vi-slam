#pragma once

#include "Camera.hpp"
#include "type_defines.hpp"

namespace vslam::data {

class Frame {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Frame(uint64_t id,
        //          const std::unordered_set<uint64_t > & observing_feature_id,
        const std::set<uint64_t>& observing_feature_id,
        const EigenAllocatedUnorderedMap<uint64_t, Vec2_t>&
            observing_feature_point_in_device,
        const Vec3_t& camera_position,
        const Quat_t& camera_attitude,
        const PinholeCameraModel& camera_parameter);
  Frame(uint64_t id,
        //          const std::unordered_set<uint64_t > & observing_feature_id,
        const std::set<uint64_t>& observing_feature_id,
        const EigenAllocatedUnorderedMap<uint64_t, Vec2_t>&
            observing_feature_point_in_device,
        const Vec3_t& camera_position,
        const Quat_t& camera_attitude);
  explicit Frame(uint64_t id);
  Frame();

  uint64_t id;
  //    std::unordered_set<uint64_t > observingFeatureId;

  /**
   * @brief
   * std::unordered_setはstd::set_intersectionがうまく行かないので、std::setにしている。
   */
  std::set<uint64_t> observingFeatureId;
  EigenAllocatedUnorderedMap<uint64_t, Vec2_t> observingFeaturePointInDevice;

  Vec3_t cameraPosition;
  Quat_t cameraAttitude;

  PinholeCameraModel cameraParameter;

 private:
};

template <typename T>
using FrameUnorderedMap = std::unordered_map<T, Frame>;

}  // namespace vslam::data
