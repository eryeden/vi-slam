//
// Created by ery on 2020/06/01.
//

#pragma once

#include <gtsam/nonlinear/ISAM2.h>

#include "BackendBase.hpp"

namespace vslam::backend {

class iSAM2Backend : public BackendBase {
 public:
  class Parameter {
   public:
    Parameter();

    gtsam::ISAM2Params AsISAM2Params();

    /// Initialization
    database_index_t reference_frame_id_;
    //  5point ransac
    double pose_initialization_ransac_threshold_;
    int32_t pose_initialization_ransac_max_iterations_;
    double pose_initialization_ransac_probability_;
    // motion only ba
    double pose_refinement_reprojection_noise_sigma_;
    double pose_refinement_landmark_position_sigma_;
    bool pose_refinement_use_previous_pose_factor_;
    double pose_refinement_previous_position_sigma_;
    double pose_refinement_previous_orientation_sigma_;

    /// Keyframe & ISAM2
    // Triangulation
    double triangulation_reprojection_error_threshold_;
    double triangulation_minimum_parallax_threshold_;
    // isam2 params
    double isam2_wildfire_threshold_;
    bool isam2_cache_linearized_factors_;
    double isam2_relinearize_threshold_;
    double isam2_relinearize_skip_;
    bool isam2_find_unused_factor_slots_;
    bool isam2_enable_partial_relinearization_check_;
    bool isam2_set_evaluate_nonlinear_error_;  // only for debugging
    bool isam2_enable_detailed_results_;       // only for debugging.
    gtsam::ISAM2Params::Factorization isam2_factorization_;

    // factor graph params
    double isam2_reprojection_noise_sigma_;
    double isam2_prior_pose_position_sigma_;
    double isam2_prior_pose_orientation_sigma_;
    int32_t isam2_iteration_number_;
    // Outlier rejection
    double optimization_reprojection_error_threshold_;
  };

 public:
  /**
   * @param map_database : Map databaseへのポインタを設定する
   */
  iSAM2Backend(const std::shared_ptr<data::ThreadsafeMapDatabase>& map_database,
               const Parameter& parameter);

  /**
   * @brief Backendの中心、この関数の呼び出しでBackend処理が全て走る
   * @return 現状のBackendState
   */
  BackendState SpinOnce();

 private:
  database_index_t latest_frame_id_;
  database_index_t latest_key_frame_id_;
  BackendState backend_state_;

  Parameter parameter_;

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
      data::FrameWeakPtr&& current_frame,
      const Parameter& parameter);

  bool InitializeISAM2(
      std::shared_ptr<gtsam::ISAM2>& isam_2,
      std::shared_ptr<data::ThreadsafeMapDatabase>& map_database,
      data::FrameWeakPtr&& reference_frame,
      data::FrameWeakPtr&& current_frame,
      double isam2_reprojection_noise_sigma,
      double isam2_prior_pose_position_sigma,
      double isam2_prior_pose_orientation_sigma,
      int32_t isam2_iteration_number);

  /**
   * @brief 2つのKeyFrame間でTriangulateする
   * @param map_database
   * @param current_key_frame
   * @param previous_key_frame
   * @param triangulated_landmarks
   * @param reprojection_error_threshold
   * @param minimum_parallax_threshold
   * @return
   */
  bool TriangulateKeyFrame(
      std::shared_ptr<data::ThreadsafeMapDatabase>& map_database,
      const data::FrameWeakPtr& current_key_frame,
      const data::FrameWeakPtr& previous_key_frame,
      vslam::EigenAllocatedUnorderedMap<database_index_t,
                                        vslam::data::LandmarkWeakPtr>&
          triangulated_landmarks,
      double reprojection_error_threshold,
      double minimum_parallax_threshold);
  /**
   * @brief
   * 新規観測Frameで観測された未初期化Landmarkのうちから、被観測Frameを探索、初期化を試みる
   * @param map_database
   * @param current_key_frame
   * @param triangulated_landmarks
   * @param reprojection_error_threshold
   * @param minimum_parallax_threshold
   * @return
   */
  bool TriangulateKeyFrame(
      std::shared_ptr<data::ThreadsafeMapDatabase>& map_database,
      const data::FrameWeakPtr& current_key_frame,
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
      double isam2_reprojection_noise_sigma,
      int32_t isam2_iteration_number,
      double reprojection_error_threshold);
};

}  // namespace vslam::backend
