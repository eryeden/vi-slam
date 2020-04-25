#include "Frame.hpp"

using namespace vslam::data;

Frame::Frame(
    uint64_t id_,
    //        const std::unordered_set<uint64_t> &observing_feature_id,
    const std::set<uint64_t>& observing_feature_id,
    const vslam::eigen_allocated_unordered_map<uint64_t, vslam::Vec2_t>&
        observing_feature_point_in_device,
    const vslam::Vec3_t& camera_position,
    const vslam::Quat_t& camera_attitude,
    const PinholeCameraModel& camera_parameter) {
  id = id_;
  observingFeatureId = observing_feature_id;
  observingFeaturePointInDevice = observing_feature_point_in_device;
  cameraPosition = camera_position;
  cameraAttitude = camera_attitude;
  cameraParameter = camera_parameter;
}

Frame::Frame(uint64_t id,
             // const std::unordered_set<uint64_t > & observing_feature_id,
             const std::set<uint64_t>& observing_feature_id,
             const eigen_allocated_unordered_map<uint64_t, Vec2_t>&
                 observing_feature_point_in_device,
             const Vec3_t& camera_position,
             const Quat_t& camera_attitude)
    : Frame(id,
            observing_feature_id,
            observing_feature_point_in_device,
            camera_position,
            camera_attitude,
            PinholeCameraModel()) {}

Frame::Frame(uint64_t id)
    : Frame(id, {}, {}, {0, 0, 0}, {0, 0, 0, 1}, PinholeCameraModel()) {
  ;
}

Frame::Frame() : Frame(std::numeric_limits<uint64_t>::max()) { ; }
