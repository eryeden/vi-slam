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

  // Landmarkの被観測情報を更新・存在しないLandmarkは更新する
  RegisterLandmarkObservation(
      map_database_, map_database_->GetFrame(map_database_->latest_frame_id_));

  // 0 Frame 目初期Frameに単位元の姿勢を設定。
  if (map_database_->latest_frame_id_ == 0) {
    latest_frame_id_ = 0;
    latest_key_frame_id_ = 0;
    // reference poseは単位元を設定
    auto ref_frame_ptr = map_database_->GetFrame(0).lock();
    if (ref_frame_ptr) {
      Pose_t identity_pose(Mat33_t::Identity(), {0, 0, 0});
      ref_frame_ptr->SetCameraPose(identity_pose);
    }
    return backend_state_;
  }

  if (backend_state_ == BackendState::BootStrap) {
    // Mapの初期化を実施
    Pose_t outpose;
    vslam::EigenAllocatedUnorderedMap<database_index_t, Vec3_t>
        output_landmark_position;
    bool status_initiation = initialization::TryInitialize(
        map_database_->GetFrame(0),
        map_database_->GetFrame(map_database_->latest_frame_id_),
        outpose,
        output_landmark_position,
        0.5,
        10);
    if (!status_initiation) {
      return backend_state_;
    }

    /**
     * @brief Poseを初期化
     */
    // current frameは推定したPoseを設定
    auto current_frame_ptr =
        map_database_->GetFrame(map_database_->latest_frame_id_).lock();
    if (current_frame_ptr) {
      current_frame_ptr->SetCameraPose(outpose);
      current_frame_ptr->is_keyframe_ = true;
    }
    // reference poseは単位元を設定
    auto ref_frame_ptr = map_database_->GetFrame(0).lock();
    if (ref_frame_ptr) {
      Pose_t identity_pose(Mat33_t::Identity(), {0, 0, 0});
      ref_frame_ptr->SetCameraPose(identity_pose);
      ref_frame_ptr->is_keyframe_ = true;
    }

    /**
     * @brief Refine
     */
    initialization::RefineInitializedMap(
        ref_frame_ptr, current_frame_ptr, output_landmark_position);

    /**
     * @brief Landmarkを初期化
     */
    for (const auto& [id, pos] : output_landmark_position) {
      if (map_database_->IsExistLandmark(id)) {
        auto lm_ptr = map_database_->GetLandmark(id).lock();
        if (lm_ptr) {
          lm_ptr->is_initialized_ = true;
          lm_ptr->SetLandmarkPosition(pos);
        } else {
          spdlog::warn("{}:{} Landmark [{}] dose exists but expired.",
                       __FILE__,
                       __FUNCTION__,
                       id);
        }
      } else {
        spdlog::warn(
            "{}:{} Landmark [{}] dose not exist.", __FILE__, __FUNCTION__, id);
      }
    }
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
void vslam::backend::iSAM2Backend::RegisterLandmarkObservation(
    std::shared_ptr<data::ThreadsafeMapDatabase>& map_database,
    const vslam::data::FrameWeakPtr& input_frame) {
  auto input_frame_ptr = input_frame.lock();
  if (input_frame_ptr) {
    for (const auto id : input_frame_ptr->observing_feature_id_) {
      if (map_database->IsExistLandmark(id)) {
        /**
         * @brief Landmarkが存在していれば被観測情報を更新する
         */
        auto lm_ptr = map_database->GetLandmark(id).lock();
        if (lm_ptr) {
          lm_ptr->SetObservedFrameIndex(input_frame_ptr->frame_id_);
        } else {
          spdlog::warn("{}:{}  Landmark exists but already expired.\n",
                       __FILE__,
                       __FUNCTION__);
        }

      } else {
        /**
         * @brief Landmarkが存在していないときは追加
         */
        database_index_t lm_id = map_database->max_landmark_id_ + 1;
        auto lm_ptr = std::unique_ptr<data::Landmark>(new data::Landmark(
            lm_id,
            std::set<database_index_t>{input_frame_ptr->frame_id_},
            {0, 0, 0},
            false,
            false));
        map_database->AddLandmark(lm_ptr);
      }
    }

  } else {
    spdlog::warn("{}:{} Input frame is expired\n", __FILE__, __FUNCTION__);
  }
}
