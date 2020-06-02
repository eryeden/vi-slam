//
// Created by ery on 2020/04/25.
//
#include "ThreadsafeContainer.hpp"

using namespace vslam::data;

ThreadsafeMapDatabase::ThreadsafeMapDatabase()
    : latest_frame_id_(0), latest_key_frame_id_(0), max_landmark_id_(0) {
  ;
}

void ThreadsafeMapDatabase::AddFrame(FrameUniquePtr& frame_ptr) {
  std::lock_guard<std::mutex> lock(mutex_map_access_);

  // 最新IDの更新
  if (latest_frame_id_ < frame_ptr->frame_id_)
    latest_frame_id_ = frame_ptr->frame_id_;
  if (frame_ptr->is_keyframe_ && latest_key_frame_id_ < frame_ptr->frame_id_)
    latest_key_frame_id_ = frame_ptr->frame_id_;

  // Frame databaseの更新
  std::shared_ptr tmp_frame_ptr{std::move(frame_ptr)};
  frame_database_[latest_frame_id_] = tmp_frame_ptr;
}
void ThreadsafeMapDatabase::EraseFrame(database_index_t frame_id) {
  std::lock_guard<std::mutex> lock(mutex_map_access_);

  if (frame_database_.count(frame_id) != 0) {  // 登録済みかチェック
    frame_database_[frame_id].reset();  // shared_ptrの所有権を放棄
    frame_database_.erase(frame_id);    // frame db から削除
  } else {
    ;
  }
}
FrameDatabaseWeak ThreadsafeMapDatabase::GetAllFrames() const {
  std::lock_guard<std::mutex> lock(mutex_map_access_);

  // 全てのshared_prtをweak_ptrに変換し出力
  FrameDatabaseWeak frame_database_weak;
  for (auto& [id, frame_shared_ptr] : frame_database_) {
    frame_database_weak[id] = frame_shared_ptr;
  }
  return frame_database_weak;
}
FrameWeakPtr ThreadsafeMapDatabase::GetFrame(
    vslam::database_index_t frame_id) const {
  std::lock_guard<std::mutex> lock(mutex_map_access_);

  if (frame_database_.count(frame_id) != 0) {  // 登録済みかチェック
    return frame_database_.at(frame_id);
  } else {
    return FrameWeakPtr();
  }
}

void ThreadsafeMapDatabase::AddLandmark(LandmarkUniquePtr& landmark_ptr) {
  std::lock_guard<std::mutex> lock(mutex_map_access_);

  if (max_landmark_id_ < landmark_ptr->landmark_id_) {
    max_landmark_id_ = landmark_ptr->landmark_id_;
  }

  std::shared_ptr tmp_landmark_ptr{std::move(landmark_ptr)};
  landmark_database_[tmp_landmark_ptr->landmark_id_] = tmp_landmark_ptr;
}
void ThreadsafeMapDatabase::EraseLandmark(database_index_t landmark_id) {
  std::lock_guard<std::mutex> lock(mutex_map_access_);

  if (landmark_database_.count(landmark_id) != 0) {  // 登録済みかチェック
    landmark_database_[landmark_id].reset();  // shared_ptrの所有権を放棄
    landmark_database_.erase(landmark_id);    // landmark db から削除
  } else {
    ;
  }
}
LandmarkDatabaseWeak ThreadsafeMapDatabase::GetAllLandmarks() const {
  std::lock_guard<std::mutex> lock(mutex_map_access_);

  // 全てのshared_prtをweak_ptrに変換し出力
  LandmarkDatabaseWeak landmark_database_weak;
  for (auto& [id, landmark_shared_ptr] : landmark_database_) {
    landmark_database_weak[id] = landmark_shared_ptr;
  }
  return landmark_database_weak;
}
LandmarkWeakPtr ThreadsafeMapDatabase::GetLandmark(
    vslam::database_index_t landmark_id) const {
  std::lock_guard<std::mutex> lock(mutex_map_access_);

  if (landmark_database_.count(landmark_id) != 0) {  // 登録済みかチェック
    return landmark_database_.at(landmark_id);
  } else {
    return LandmarkSharedPtr();
  }
}
bool ThreadsafeMapDatabase::IsExistLandmark(
    database_index_t landmark_id) const {
  std::lock_guard<std::mutex> lock(mutex_map_access_);

  if (landmark_database_.count(landmark_id) != 0) {  // 登録済みかチェック
    return true;
  } else {
    return false;
  }
}

void ThreadsafeMapDatabase::Clear() {
  // 全Frameの所有権を開放する
  for (auto [id, frame_ptr] : frame_database_) {
    frame_ptr.reset();
  }
  frame_database_.clear();

  // 全Landmarkの所有権を開放する
  for (auto [id, landmark_ptr] : landmark_database_) {
    landmark_ptr.reset();
  }
  landmark_database_.clear();

  latest_key_frame_id_ = 0;
  latest_frame_id_ = 0;
}
