#include "Landmark.hpp"

using namespace vslam;
using namespace vslam::data;

Landmark::Landmark(vslam::database_index_t id,
                   const std::set<database_index_t>& observed_frame_id,
                   const vslam::Vec3_t& position_in_world,
                   bool is_outlier,
                   bool is_initialized)
    : landmark_id_(id),
      observed_frame_id_(observed_frame_id),
      position_in_world_(position_in_world),
      is_outlier_(is_outlier),
      is_initialized_(is_initialized) {
  ;
}

Landmark::Landmark()
    : Landmark(0, std::set<database_index_t>(), {0, 0, 0}, false, false) {}

Landmark::Landmark(const Landmark& landmark)
    : Landmark(landmark.landmark_id_,
               landmark.observed_frame_id_,
               landmark.position_in_world_,
               landmark.is_outlier_,
               landmark.is_initialized_) {
  ;
}

void Landmark::SetObservedFrameIndex(vslam::database_index_t frame_index) {
  std::lock_guard<std::mutex> lock(mutex_observation_);
  observed_frame_id_.insert(frame_index);
}
std::set<database_index_t> Landmark::GetAllObservedFrameIndex() const {
  std::lock_guard<std::mutex> lock(mutex_observation_);
  return observed_frame_id_;
}
void Landmark::SetLandmarkPosition(const vslam::Vec3_t& position_in_world) {
  std::lock_guard<std::mutex> lock(mutex_position_);
  position_in_world_ = position_in_world;
}
vslam::Vec3_t Landmark::GetLandmarkPosition() const {
  std::lock_guard<std::mutex> lock(mutex_position_);
  return position_in_world_;
}
