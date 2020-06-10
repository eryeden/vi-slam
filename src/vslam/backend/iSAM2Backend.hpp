//
// Created by ery on 2020/06/01.
//

#pragma once

#include <gtsam/nonlinear/ISAM2.h>

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

  std::shared_ptr<gtsam::ISAM2> isam_2_ptr_;

  /**
   * @brief
   * Landmarkの被観測情報を更新する。Landmarkが存在しない場合は新規登録し、被観測情報を追加する。
   * @param map_database
   * @param input_frame
   */
  void RegisterLandmarkObservation(
      std::shared_ptr<data::ThreadsafeMapDatabase>& map_database,
      const data::FrameWeakPtr& input_frame);

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
      data::FrameWeakPtr&& current_frame);

  bool InitializeISAM2(
      std::shared_ptr<gtsam::ISAM2>& isam_2,
      std::shared_ptr<data::ThreadsafeMapDatabase>& map_database,
      data::FrameWeakPtr&& reference_frame,
      data::FrameWeakPtr&& current_frame);

  bool TriangulateKeyFrame(
      std::shared_ptr<data::ThreadsafeMapDatabase>& map_database,
      const data::FrameWeakPtr& current_key_frame,
      const data::FrameWeakPtr& previous_key_frame,
      vslam::EigenAllocatedUnorderedMap<database_index_t,
                                        vslam::data::LandmarkWeakPtr>&
          triangulated_landmarks,
      double reprojection_error_threshold,
      double minimum_parallax_threshold);


  bool UpdateISAMObservation(
      std::shared_ptr<gtsam::ISAM2>& isam_2,
      std::shared_ptr<data::ThreadsafeMapDatabase>& map_database,
      vslam::EigenAllocatedUnorderedMap<database_index_t,
                                        vslam::data::LandmarkWeakPtr>&
          triangulated_landmarks,
      double reprojection_error_threshold);
};

}  // namespace vslam::backend
