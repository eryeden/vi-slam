//
// Created by ery on 2020/06/01.
//

#include "iSAM2Backend.hpp"

#include "Initialization.hpp"
#include "spdlog/spdlog.h"
#include "type_defines.hpp"

vslam::backend::iSAM2Backend::iSAM2Backend(
    const std::shared_ptr<data::ThreadsafeMapDatabase>& map_database)
    : BackendBase(map_database),
      backend_state_(BackendState::BootStrap),
      latest_frame_id_(std::numeric_limits<database_index_t>::max()),
      latest_key_frame_id_(std::numeric_limits<database_index_t>::max()) {
  ;
}
vslam::backend::BackendState vslam::backend::iSAM2Backend::SpinOnce() {
  // 更新があるときのみ以下を実行する。
  if (latest_frame_id_ == map_database_->latest_frame_id_) {
    return backend_state_;
  }

  // 初回は何もしない。
  if (map_database_->latest_frame_id_ == 0) {
    latest_frame_id_ = 0;
    latest_key_frame_id_ = 0;
    return backend_state_;
  }

  if (backend_state_ == BackendState::BootStrap) {
    // Mapの初期化を実施
    Pose_t outpose;
    vslam::EigenAllocatedUnorderedMap<database_index_t, Vec3_t>
        output_landmark_position;
    initialization::TryInitialize(
        map_database_->GetFrame(0),
        map_database_->GetFrame(map_database_->latest_frame_id_),
        outpose,
        output_landmark_position,
        0.5);

  } else if (backend_state_ == BackendState::Nominal) {
    // KeyFrameの時：

    // KeyFrame以外の時：

  } else {
    spdlog::warn("{}:{} Unrecognized Backend status:{}",
                 __FILE__,
                 __FUNCTION__,
                 backend_state_);
  }

  // frame_idの更新
  latest_frame_id_ = map_database_->latest_frame_id_;
  latest_key_frame_id_ = map_database_->latest_key_frame_id_;

  return backend_state_;
}
