//
// Created by ery on 2020/06/19.
//

#pragma once

#include <gtsam/nonlinear/ISAM2.h>

#include "BackendBase.hpp"
#include "iSAM2Backend.hpp"

namespace vslam::backend {

class ContinuousDetectorBackend : public BackendBase {
 public:

 public:
  /**
   * @param map_database : Map databaseへのポインタを設定する
   */
  ContinuousDetectorBackend(
      const std::shared_ptr<data::ThreadsafeMapDatabase>& map_database,
      const iSAM2Backend::Parameter& parameter);

  /**
   * @brief Backendの中心、この関数の呼び出しでBackend処理が全て走る
   * @return 現状のBackendState
   */
  BackendState SpinOnce();

 private:
  database_index_t latest_frame_id_;
  database_index_t latest_key_frame_id_;
  BackendState backend_state_;

  iSAM2Backend::Parameter parameter_;

  std::shared_ptr<gtsam::ISAM2> isam_2_ptr_;

  /**
   * @brief Mapの初期化を実施
   * @param map_database[in,out]
   * @param reference_frame[in,out] : 初期化の基準となるFrame
   * @param current_frame[in,out] :
   * @return
   */
  bool MapInitialization(
      std::shared_ptr<data::ThreadsafeMapDatabase>& map_database,
      data::FrameWeakPtr&& reference_frame,
      data::FrameWeakPtr&& current_frame,
      const iSAM2Backend::Parameter& parameter);
};

}  // namespace vslam::backend
