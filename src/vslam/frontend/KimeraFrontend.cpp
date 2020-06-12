//
// Created by ery on 2020/04/30.
//

#include "KimeraFrontend.hpp"

#include <spdlog/spdlog.h>

#include "OpenCVUtilities.hpp"

using namespace vslam::data;
using namespace vslam::frontend;

KimeraFrontendInput::KimeraFrontendInput()
    : KimeraFrontendInput(0,
                          cv::Mat(),
                          cv::Mat(),
                          std::unique_ptr<CameraModelBase>()) {
  ;
}
KimeraFrontendInput::KimeraFrontendInput(
    double timestamp,
    const cv::Mat& frame,
    const cv::Mat& mask,
    const std::unique_ptr<data::CameraModelBase>& camera_model)
    : timestamp_(timestamp) {
  frame_ = frame;
  mask_ = mask;
  if (camera_model) {
    camera_model_ptr_ =
        std::unique_ptr<data::CameraModelBase>(camera_model->Clone());
  }
}

KimeraFrontendInput::KimeraFrontendInput(
    const KimeraFrontendInput& kimera_frontend_input)
    : KimeraFrontendInput(kimera_frontend_input.timestamp_,
                          kimera_frontend_input.frame_,
                          kimera_frontend_input.mask_,
                          kimera_frontend_input.camera_model_ptr_) {}

KimeraFrontendInput& KimeraFrontendInput::operator=(
    const KimeraFrontendInput& kimera_frontend_input) {
  timestamp_ = kimera_frontend_input.timestamp_;
  frame_ = kimera_frontend_input.frame_;
  mask_ = kimera_frontend_input.mask_;
  if (kimera_frontend_input.camera_model_ptr_ != nullptr) {
    camera_model_ptr_.reset(kimera_frontend_input.camera_model_ptr_->Clone());
  }
  return *this;
}

/**
 * @brief Kimera-VIOベースの単眼Frontend
 * @param threadsafe_map_database
 */
KimeraFrontend::KimeraFrontend(
    const std::shared_ptr<data::ThreadsafeMapDatabase>& threadsafe_map_database,
    const std::shared_ptr<feature::FeatureDetectorBase>& feature_detector,
    const std::shared_ptr<feature::FeatureTrackerBase>& feature_tracker,
    const std::shared_ptr<verification::FeatureVerification5PointRANSAC>&
        feature_verification,
    Parameter parameter)
    : FrontendBase(threadsafe_map_database),
      is_first_frame_(true),
      parameter_(parameter) {
  feature_detector_ = feature_detector;
  feature_tracker_ = feature_tracker;
  feature_verification_ = feature_verification;

  // input check
  if (feature_tracker_ == nullptr) {
    spdlog::warn(
        "{}:{} Null input of feature tracker.", __FILE__, __FUNCTION__);
  }
  // input check
  if (feature_detector_ == nullptr) {
    spdlog::warn(
        "{}:{} Null input of feature detector.", __FILE__, __FUNCTION__);
  }
  // input check
  if (feature_verification_ == nullptr) {
    spdlog::warn(
        "{}:{} Null input of feature verification.", __FILE__, __FUNCTION__);
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
   * 1. Feature detection
   * 2. FrameDBにFirst Keyframeとして登録
   * (LandmarkDBへの特徴点登録は、２Frame以降のFrameVerificationを実行した後に行う)
   *
   * ## 2Frame目以降の場合
   * 1. 前FrameのFeature pointをTrackする
   * 2. KeyFrameかどうかの判定を行う
   * 3. ===
   *
   * ### KeyFrameでない場合
   * 3.1. 検出LandmarkをLandmark DBに、FrameをFrameDBに登録
   * (2つめのKeyFrameが出現し、Verificationが完了するまでは、LandmarkDBへの更新、登録を行わない)
   *
   * ### KeyFrameの場合
   * 3.1. Verificationを実施、5-pointRANSACや、Feature ageによる特徴点削除を行う
   * 3.2. 特徴点の不足、分布の偏りがあれば、特徴点の追加検出を行う。
   *
   */

  if (is_first_frame_) {
    ////////////////////////// Process the first frame /////////////////////////
    is_first_frame_ = false;

    auto processed_frame = ProcessFirstFrame(frontend_input);
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
        ProcessFrame(frontend_input, last_frame_, last_keyframe_);
    last_frame_ =
        std::make_shared<data::Frame>(processed_frame);  // Frame copied

    if (last_frame_->is_keyframe_) {
      last_keyframe_ = last_frame_;
    }
    auto tmp_frame_ptr =
        std::make_unique<data::Frame>(processed_frame);  // Frame copied
    map_database_->AddFrame(tmp_frame_ptr);
  }

  last_input_ = frontend_input;
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
  FeatureBearingDatabase feature_bearing_database;
  std::set<database_index_t> feature_id_database;

  feature_detector_->UpdateDetection(feature_position_database,
                                     feature_age_database,
                                     frontend_input.frame_,
                                     frontend_input.mask_);

  // Update Observed id and feature bearing information
  for (const auto& [id, pos] : feature_position_database) {
    feature_id_database.insert(id);
    //    feature_bearing_database[id] =
    //        frontend_input.camera_model_ptr_->Unproject(pos);
    try {
      feature_bearing_database[id] =
          frontend_input.camera_model_ptr_->Unproject(pos);
    } catch (data::ProjectionErrorException& exception) {
      spdlog::warn("Unprojection error: {}", exception.what());
      feature_bearing_database[id] = {0, 0, 1};
    }
  }

  return Frame(0,
               frontend_input.timestamp_,
               true,
               frontend_input.camera_model_ptr_,
               feature_id_database,
               feature_position_database,
               feature_bearing_database,
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
  FeatureBearingDatabase
      feature_bearing_database;  // =
                                 // last_frame->observing_feature_bearing_in_camera_frame_;
  std::set<database_index_t> feature_id_database;
  //      last_frame->observing_feature_id_;

  spdlog::info("########### Feature Tracking ###########");
  int32_t input_feature_number = feature_position_database.size();
  spdlog::info(
      "{} : Input feature number {}", __FUNCTION__, input_feature_number);

  // Track feature
  //  feature_tracker_->Track(feature_position_database,
  //                                       feature_age_database,
  //                                       last_input_.frame_,
  //                                       frontend_input.frame_);
  feature_tracker_->Track(
      feature_position_database, feature_age_database, frontend_input.frame_);

  // update id list and feature bearing vector
  for (const auto& [id, pos] : feature_position_database) {
    feature_id_database.insert(id);
    try {
      feature_bearing_database[id] =
          frontend_input.camera_model_ptr_->Unproject(pos);
    } catch (data::ProjectionErrorException& exception) {
      spdlog::warn(
          "{} : Unprojection error: {}", __FUNCTION__, exception.what());
      feature_bearing_database[id] = {0, 0, 1};
    }
  }
  int32_t tracked_feature_number = feature_position_database.size();
  spdlog::info("{} : Tracked feature number {}, PassRate {}",
               __FUNCTION__,
               tracked_feature_number,
               static_cast<double>(tracked_feature_number) /
                   static_cast<double>(input_feature_number) * 100.0);

  // landmark ageが2歳以上のものをカウントする
  uint32_t aged_landmark_number = 0;
  for (const auto& itr : feature_age_database) {
    if (itr.second >= parameter_.counting_feature_age_) aged_landmark_number++;
  }

  // check the number of landmark
  bool features_low_number = false;
  //  features_low_number =
  //      (aged_landmark_number < keyframe_feature_number_threshold_);
  features_low_number =
      (aged_landmark_number < parameter_.low_keyframe_feature_number_);

  if ((frontend_input.timestamp_ - last_keyframe->timestamp_) <
      parameter_.minimum_keyframe_interval_) {
    features_low_number = false;
  }

  //    features_low_number = (feature_position_database.size() <
  //    keyframe_feature_number_threshold_);

  // Select keyframe or not.
  //  if ((feature_position_database.size() <
  //  keyframe_feature_number_threshold_) ||
  //      ((frontend_input.timestamp_ - last_keyframe->timestamp_) >
  //       keyframe_interval_threshold_))
  if (features_low_number ||
      ((frontend_input.timestamp_ - last_keyframe->timestamp_) >
       parameter_.keyframe_interval_)) {
    spdlog::info(
        "Keyframe detected. Feature[qty] : {}/{}, Interval[s] : {:.3}/{:.3}",
        feature_position_database.size(),
        parameter_.low_keyframe_feature_number_,
        (frontend_input.timestamp_ - last_keyframe->timestamp_),
        parameter_.keyframe_interval_);

    // Increment feature age
    for (auto& itr : feature_age_database) {
      itr.second++;
    }

    // Verification
    data::Frame tmp_frame(0,
                          0,
                          true,
                          frontend_input.camera_model_ptr_,
                          feature_id_database,
                          feature_position_database,
                          feature_bearing_database,
                          feature_age_database);
    auto verified_frame =
        feature_verification_->RemoveOutlier(*last_keyframe_, tmp_frame);
    feature_id_database = verified_frame.observing_feature_id_;
    feature_position_database =
        verified_frame.observing_feature_point_in_device_;
    feature_bearing_database =
        verified_frame.observing_feature_bearing_in_camera_frame_;
    feature_age_database = verified_frame.feature_point_age_;

    int32_t verified_feature_number = feature_position_database.size();
    spdlog::info("{} : Verified feature number {}, PassRate {}",
                 __FUNCTION__,
                 verified_feature_number,
                 static_cast<double>(verified_feature_number) /
                     static_cast<double>(input_feature_number) * 100.0);

    // Feature detection
    feature_detector_->UpdateDetection(feature_position_database,
                                       feature_age_database,
                                       frontend_input.frame_,
                                       frontend_input.mask_);
    for (const auto& [id, pos] : feature_position_database) {
      feature_id_database.insert(id);
      //      feature_bearing_database[id] =
      //          frontend_input.camera_model_ptr_->Unproject(pos);

      try {
        feature_bearing_database[id] =
            frontend_input.camera_model_ptr_->Unproject(pos);
      } catch (data::ProjectionErrorException& exception) {
        spdlog::warn("Unprojection error: {}", exception.what());
        feature_bearing_database[id] = {0, 0, 1};
      }
    }

    int32_t detected_feature_number = feature_position_database.size();
    spdlog::info("{} : Detected feature number {}, PassRate {}",
                 __FUNCTION__,
                 detected_feature_number,
                 static_cast<double>(detected_feature_number) /
                     static_cast<double>(input_feature_number) * 100.0);

    return Frame(last_frame_->frame_id_ + 1,
                 frontend_input.timestamp_,
                 true,
                 frontend_input.camera_model_ptr_,
                 feature_id_database,
                 feature_position_database,
                 feature_bearing_database,
                 feature_age_database);

  } else {
    //    data::Frame tmp_frame(0,
    //                          0,
    //                          true,
    //                          frontend_input.camera_model_ptr_,
    //                          feature_id_database,
    //                          feature_position_database,
    //                          feature_bearing_database,
    //                          feature_age_database);
    //    auto verified_frame =
    //        feature_verification_->RemoveOutlier(*last_keyframe_, tmp_frame);
    //    feature_id_database = verified_frame.observing_feature_id_;
    //    feature_position_database =
    //        verified_frame.observing_feature_point_in_device_;
    //    feature_bearing_database =
    //        verified_frame.observing_feature_bearing_in_camera_frame_;
    //    feature_age_database = verified_frame.feature_point_age_;

    return Frame(last_frame_->frame_id_ + 1,
                 frontend_input.timestamp_,
                 false,
                 frontend_input.camera_model_ptr_,
                 feature_id_database,
                 feature_position_database,
                 feature_bearing_database,
                 feature_age_database);
  }
}
