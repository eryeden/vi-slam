//
// Created by ery on 2020/06/01.
//

#pragma once

#include "BackendBase.hpp"

namespace vslam::backend {

class iSAM2Backend : public BackendBase {
 public:
  /**
   * @param map_database : Map databaseへのポインタを設定する
   */
  iSAM2Backend(
      const std::shared_ptr<data::ThreadsafeMapDatabase>& map_database);

  /**
   * @brief Backendの中心、この関数の呼び出しでBackend処理が全て走る
   * @return 現状のBackendState
   */
  BackendState SpinOnce();

 private:
  database_index_t latest_frame_id_;
  database_index_t latest_key_frame_id_;

  BackendState backend_state_;

  /**
   * @brief
   * Landmarkの被観測情報を更新する。Landmarkが存在しない場合は新規登録し、被観測情報を追加する。
   * @param map_database
   * @param input_frame
   */
  void RegisterLandmarkObservation(
      std::shared_ptr<data::ThreadsafeMapDatabase>& map_database,
      const data::FrameWeakPtr& input_frame);
};

}  // namespace vslam::backend
