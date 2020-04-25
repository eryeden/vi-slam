#include "Landmark.hpp"

using namespace vslam::data;

Landmark::Landmark(
    uint64_t id_,
    //        const std::unordered_set<uint64_t> & observed_frame_id,
    const std::set<uint64_t>& observed_frame_id,
    const Vec3_t& position_in_world,
    bool is_outlier,
    bool is_tracking,
    bool is_initialized) {
  id = id_;

  observedFrameId = observed_frame_id;
  positionInWorld = position_in_world;
  isOutlier = is_outlier;
  isTracking = is_tracking;
  isInitialized = is_initialized;
}
Landmark::Landmark(uint64_t id_)
    : Landmark(id_, {}, {0, 0, 0}, false, false, false) {
  ;
}
Landmark::Landmark() : Landmark(std::numeric_limits<uint64_t>::max()) { ; }
