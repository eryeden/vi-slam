//
// Created by ery on 2020/04/30.
//

#include "KimeraFrontend.hpp"

#include <spdlog/spdlog.h>

#include "OpenCVUtilities.hpp"

using namespace vslam::data;
using namespace vslam::frontend;

/**
 * @brief Kimera-VIOベースの単眼Frontend
 * @param threadsafe_map_database
 */
KimeraFrontend::KimeraFrontend(
    const std::shared_ptr<data::ThreadsafeMapDatabase>& threadsafe_map_database,
    const std::shared_ptr<feature::FeatureDetectorShiTomasi>&
        feature_detector_shi_tomasi,
    const std::shared_ptr<feature::FeatureTrackerLucasKanade>&
        feature_tracker_lucas_kanade,
    double keyframe_interval_threshold,
    uint32_t keyframe_feature_number_threshold)
    : FrontendBase(threadsafe_map_database),
      is_first_frame_(true),
      keyframe_interval_threshold_(keyframe_interval_threshold),
      keyframe_feature_number_threshold_(keyframe_feature_number_threshold) {
  feature_detector_shi_tomasi_ = feature_detector_shi_tomasi;
  feature_tracker_lucas_kanade_ = feature_tracker_lucas_kanade;

  // input check
  if (feature_tracker_lucas_kanade_ == nullptr) {
    spdlog::warn(
        "{}:{} Null input of feature tracker.", __FILE__, __FUNCTION__);
  }
  // input check
  if (feature_detector_shi_tomasi_ == nullptr) {
    spdlog::warn(
        "{}:{} Null input of feature detector.", __FILE__, __FUNCTION__);
  }
}

/**
 * @brief Implementation of Kimera-based Vision-Frontend
 * @details
 * Frontendのメインループとなるので、ここを起点にFrontendの全処理が記述される
 * @param input_image
 * @return
 */
FrontendStatus KimeraFrontend::Feed(const KimeraFrontendInput& frontend_input) {
  /**
   * @brief 処理フロー
   * @details
   * ## 初期フレームの場合
   * 1. 画像をUndistort
   * 2. Feature detection
   * 3. FrameDBにFirst Keyframeとして登録
   * (LandmarkDBへの特徴点登録は、２Frame以降のFrameVerificationを実行した後に行う)
   *
   * ## 2Frame目以降の場合
   * 1. 画像をUndistort
   * 2. 前FrameのFeature pointをTrackする
   * 3. KeyFrameかどうかの判定を行う
   * 4. ===
   *
   * ### KeyFrameでない場合
   * 4.1. 検出LandmarkをLandmark DBに、FrameをFrameDBに登録
   * (2つめのKeyFrameが出現し、Verificationが完了するまでは、LandmarkDBへの更新、登録を行わない)
   *
   * ### KeyFrameの場合
   * 4.1. Verificationを実施、5-pointRANSACや、Feature ageによる特徴点削除を行う
   * 4.2. 特徴点の不足、分布の偏りがあれば、特徴点の追加検出を行う。
   *
   */

  //! Undistort
  cv::Mat undistorted_image;
  cv::undistort(frontend_input.frame_,
                undistorted_image,
                utility::ConvertEigenMatToCVMat(
                    frontend_input.camera_model_.GetIntrinsicMatrix()),
                frontend_input.camera_model_.GetDistortionParameters());
  KimeraFrontendInput undistorted_frontend_input = frontend_input;
  undistorted_frontend_input.frame_ = undistorted_image;

  if (is_first_frame_) {
    ////////////////////////// Process the first frame /////////////////////////
    is_first_frame_ = false;

    auto processed_frame = ProcessFirstFrame(undistorted_frontend_input);
    last_frame_ =
        std::make_shared<data::Frame>(processed_frame);  // Frame copied

    if (last_frame_->is_keyframe_) {
      last_keyframe_ = last_frame_;
    }
    auto tmp_frame_ptr =
        std::make_unique<data::Frame>(processed_frame);  // Frame copied
    map_database_->AddFrame(tmp_frame_ptr);

  } else {
    ////////////////////////// Process the second or later frame ///////////////
    auto processed_frame =
        ProcessFrame(undistorted_frontend_input, last_frame_, last_keyframe_);
    last_frame_ =
        std::make_shared<data::Frame>(processed_frame);  // Frame copied

    if (last_frame_->is_keyframe_) {
      last_keyframe_ = last_frame_;
    }
    auto tmp_frame_ptr =
        std::make_unique<data::Frame>(processed_frame);  // Frame copied
    map_database_->AddFrame(tmp_frame_ptr);
  }

  last_input_ = undistorted_frontend_input;
  return FrontendStatus();
}

/**
 * @brief 一番最初のフレームの処理
 * @param frontend_input
 * @return
 */
Frame KimeraFrontend::ProcessFirstFrame(
    const KimeraFrontendInput& frontend_input) {
  /**
   * @brief 処理内容、一番最初のFrameを生成する
   * @details
   * input : 補正済み画像
   * 1. 特徴点の検出
   * 2. KeyFrame生成、初期位置設定
   */

  // detect features
  FeatureAgeDatabase feature_age_database;
  FeaturePositionDatabase feature_position_database;
  std::set<database_index_t> feature_id_database;

  feature_detector_shi_tomasi_->UpdateDetection(
      feature_position_database, feature_age_database, frontend_input.frame_);
  for (const auto& [id, pos] : feature_position_database) {
    feature_id_database.insert(id);
  }

  return Frame(0,
               frontend_input.timestamp_,
               true,
               frontend_input.camera_model_,
               feature_id_database,
               feature_position_database,
               feature_age_database);
}

/**
 * @brief 2Frame以降の処理
 * @param frontend_input
 * @param last_frame
 * @param last_keyframe
 * @return
 */
Frame KimeraFrontend::ProcessFrame(const KimeraFrontendInput& frontend_input,
                                   const FrameSharedPtr& last_frame,
                                   const FrameSharedPtr& last_keyframe) {
  /**
   * @brief 処理内容
   * @details
   * input : 2枚目以降のKimeraFrontendInput
   * 1. Feature tracking
   * 2. Keyframe selection
   * 3. (Keyframeなら)Verification
   * 4. (Keyframeなら)FeatureDetection
   * 5. Pose推定(あとで実装)
   */

  FeatureAgeDatabase feature_age_database = last_frame->feature_point_age_;
  FeaturePositionDatabase feature_position_database =
      last_frame->observing_feature_point_in_device_;
  std::set<database_index_t> feature_id_database =
      last_frame->observing_feature_id_;

  // Track feature
  feature_tracker_lucas_kanade_->Track(feature_position_database,
                                       feature_age_database,
                                       last_input_.frame_,
                                       frontend_input.frame_);

  // Select keyframe or not.
  if ((feature_position_database.size() < keyframe_feature_number_threshold_) ||
      ((frontend_input.timestamp_ - last_keyframe->timestamp_) <
       keyframe_interval_threshold_)) {
    // Verification
    // TODO : Implement this.

    // Feature detection
    feature_detector_shi_tomasi_->UpdateDetection(
        feature_position_database, feature_age_database, frontend_input.frame_);
    for (const auto& [id, pos] : feature_position_database) {
      feature_id_database.insert(id);
    }

    return Frame(last_frame_->frame_id_ + 1,
                 frontend_input.timestamp_,
                 true,
                 frontend_input.camera_model_,
                 feature_id_database,
                 feature_position_database,
                 feature_age_database);

  } else {
    return Frame(last_frame_->frame_id_ + 1,
                 frontend_input.timestamp_,
                 false,
                 frontend_input.camera_model_,
                 feature_id_database,
                 feature_position_database,
                 feature_age_database);
  }
}
