#pragma once

#include "type_defines.hpp"

namespace vslam::data {

class Landmark {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Landmark(
      uint64_t id,
      //             const std::unordered_set<uint64_t> & observed_frame_id,
      const std::set<uint64_t>& observed_frame_id,
      const Vec3_t& position_in_world,
      bool is_outlier,
      bool is_tracking,
      bool is_initialized);
  explicit Landmark(uint64_t id);
  Landmark();

  uint64_t id;
  //        std::unordered_set<uint64_t> observedFrameId;
  /**
   * @brief
   * std::unordered_setはstd::set_intersectionがうまく行かないので、std::setにしている。
   */
  std::set<uint64_t> observedFrameId;
  Vec3_t positionInWorld;

  bool isInitialized;
  bool isOutlier;
  bool isTracking;

 private:
};

template <typename T>
using LandmarkUnorderedMap = std::unordered_map<T, Landmark>;

}  // namespace vslam::data