#include "Frame.hpp"

using namespace vslam;
using namespace vslam::data;

Frame::Frame(
    database_index_t id,
    double timestamp,
    bool is_keyframe,
    const std::unique_ptr<CameraModelBase>& camera_model,
    const std::set<database_index_t>& observing_feature_id,
    const FeaturePositionDatabase& observing_feature_points_in_device,
    const FeatureBearingDatabase& observing_feature_bearing_in_camera_frame,
    const FeatureAgeDatabase& feature_point_age)
    : frame_id_(id),
      timestamp_(timestamp),
      is_keyframe_(is_keyframe),
      camera_model_(std::move(camera_model->Clone())),
      observing_feature_id_(observing_feature_id),
      observing_feature_point_in_device_(observing_feature_points_in_device),
      observing_feature_bearing_in_camera_frame_(
          observing_feature_bearing_in_camera_frame),
      feature_point_age_(feature_point_age) {
  ;
}

Frame::Frame()
    : Frame(0,
            0,
            false,
            std::unique_ptr<CameraModelBase>(),
            std::set<database_index_t>(),
            FeaturePositionDatabase(),
            FeatureBearingDatabase(),
            FeatureAgeDatabase()) {}

Frame::Frame(const Frame& frame)
    : Frame(frame.frame_id_,
            frame.timestamp_,
            frame.is_keyframe_,
            std::unique_ptr<CameraModelBase>(
                (frame.camera_model_) ? frame.camera_model_->Clone() : nullptr),
            frame.observing_feature_id_,
            frame.observing_feature_point_in_device_,
            frame.observing_feature_bearing_in_camera_frame_,
            frame.feature_point_age_) {
  camera_pose_ = frame.camera_pose_;
  landmarks_ = frame.landmarks_;
  internal_materials_ = frame.internal_materials_;
}

void Frame::SetCameraPose(const Pose_t& pose) {
  std::lock_guard<std::mutex> lock(mutex_camera_pose_);
  camera_pose_ = pose;
}
Vec3_t Frame::GetCameraPosition() const {
  std::lock_guard<std::mutex> lock(mutex_camera_pose_);
  return camera_pose_.translation();
}
Rot_t Frame::GetCameraOrientation() const {
  std::lock_guard<std::mutex> lock(mutex_camera_pose_);
  return camera_pose_.rotationMatrix();
}
Pose_t Frame::GetCameraPose() const {
  std::lock_guard<std::mutex> lock(mutex_camera_pose_);
  return camera_pose_;
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
