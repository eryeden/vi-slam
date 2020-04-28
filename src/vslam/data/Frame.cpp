#include "Frame.hpp"

using namespace vslam;
using namespace vslam::data;

Frame::Frame(database_index_t id,
             double timestamp,
             bool is_keyframe,
             const PinholeCameraModel& camera_parameters,
             const std::set<database_index_t>& observing_feature_id,
             const EigenAllocatedUnorderedMap<database_index_t, Vec2_t>&
                 observing_feature_points_in_device)
    : frame_id_(id),
      timestamp_(timestamp),
      is_keyframe_(is_keyframe),
      camera_parameter_(camera_parameters),
      observing_feature_id_(observing_feature_id),
      observing_feature_point_in_device_(observing_feature_points_in_device) {
  ;
}

void Frame::SetCameraPose(const vslam::Vec3_t& position,
                          const vslam::Quat_t& orientation) {
  std::lock_guard<std::mutex> lock(mutex_camera_pose_);
  camera_position_ = position;
  camera_orientation_ = orientation;
}
vslam::Vec3_t Frame::GetCameraPosition() const {
  std::lock_guard<std::mutex> lock(mutex_camera_pose_);
  return camera_position_;
}
vslam::Quat_t Frame::GetCameraOrientation() const {
  std::lock_guard<std::mutex> lock(mutex_camera_pose_);
  return camera_orientation_;
}
void Frame::GetCameraPose(vslam::Vec3_t& position,
                          vslam::Quat_t& orientation) const {
  std::lock_guard<std::mutex> lock(mutex_camera_pose_);
  position = camera_position_;
  orientation = camera_orientation_;
}

void Frame::SetLandmark(const LandmarkSharedPtr& landmark) {
  std::lock_guard<std::mutex> lock(mutex_landmark_);
  landmarks_[landmark->landmark_id_] = landmark;
}
void Frame::EraseLandmark(vslam::database_index_t landmark_index) {
  std::lock_guard<std::mutex> lock(mutex_landmark_);
  landmarks_.erase(landmark_index);
}
LandmarkDatabaseWeak Frame::GetAllLandmarks() const {
  std::lock_guard<std::mutex> lock(mutex_landmark_);
  return landmarks_;
}
LandmarkWeakPtr Frame::GetLandmark(vslam::database_index_t landmark_id) const {
  std::lock_guard<std::mutex> lock(mutex_landmark_);
  return landmarks_.at(landmark_id);
}
