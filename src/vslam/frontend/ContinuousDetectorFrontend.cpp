//
// Created by ery on 2020/06/18.
//

#include "ContinuousDetectorFrontend.hpp"

#include <spdlog/spdlog.h>

using namespace vslam;
using namespace vslam::data;

vslam::frontend::ContinuousDetectorFrontend::ContinuousDetectorFrontend(
    const std::shared_ptr<data::ThreadsafeMapDatabase>& threadsafe_map_database,
    const std::shared_ptr<feature::FeatureDetectorBase>& feature_detector,
    const std::shared_ptr<feature::FeatureTrackerBase>& feature_tracker,
    const std::shared_ptr<verification::FeatureVerification5PointRANSAC>&
        feature_verification,
    vslam::frontend::ContinuousDetectorFrontend::Parameter parameter)
    : FrontendBase(threadsafe_map_database),
      is_first_frame_(true),
      parameter_(parameter) {
  feature_detector_ = feature_detector;
  feature_tracker_ = feature_tracker;
  feature_verification_ = feature_verification;

  // input check
  if (feature_tracker_ == nullptr) {
    spdlog::error("{} Null input of feature tracker.", __FUNCTION__);
  }
  // input check
  if (feature_detector_ == nullptr) {
    spdlog::warn("{} Null input of feature detector.", __FUNCTION__);
  }
  // input check
  if (feature_verification_ == nullptr) {
    spdlog::warn("{} Null input of feature verification.", __FUNCTION__);
  }
}

vslam::frontend::FrontendStatus
vslam::frontend::ContinuousDetectorFrontend::Feed(
    const vslam::frontend::KimeraFrontendInput& frontend_input) {
  if (is_first_frame_) {
    ////////////////////////// Process the first frame /////////////////////////
    is_first_frame_ = false;

    auto processed_frame = ProcessFirstFrame(frontend_input);
    last_frame_ =
        std::make_shared<data::Frame>(processed_frame);  // Frame copied

    auto tmp_frame_ptr =
        std::make_unique<data::Frame>(processed_frame);  // Frame copied
    map_database_->AddFrame(tmp_frame_ptr);

  } else {
    ////////////////////////// Process the second or later frame ///////////////
    auto processed_frame = ProcessFrame(frontend_input, last_frame_);
    last_frame_ =
        std::make_shared<data::Frame>(processed_frame);  // Frame copied

    auto tmp_frame_ptr =
        std::make_unique<data::Frame>(processed_frame);  // Frame copied
    map_database_->AddFrame(tmp_frame_ptr);
  }

  last_input_ = frontend_input;
  return FrontendStatus();
}
vslam::data::Frame
vslam::frontend::ContinuousDetectorFrontend::ProcessFirstFrame(
    const vslam::frontend::KimeraFrontendInput& frontend_input) {
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
vslam::data::Frame vslam::frontend::ContinuousDetectorFrontend::ProcessFrame(
    const vslam::frontend::KimeraFrontendInput& frontend_input,
    const vslam::data::FrameSharedPtr& last_frame) {
  /**
   * @brief 処理内容
   * @details
   * input : 2枚目以降のKimeraFrontendInput
   * 1. Feature tracking
   * 2. Feature detection
   * 3. Verification
   */

  /**
   * @brief logging
   */
  InternalMaterials internal_materials;
  // Log input feature
  internal_materials.features_pre_frame_ =
      last_frame->observing_feature_point_in_device_;

  FeatureAgeDatabase feature_age_database = last_frame->feature_point_age_;
  FeaturePositionDatabase feature_position_database =
      last_frame->observing_feature_point_in_device_;
  FeatureBearingDatabase feature_bearing_database;
  std::set<database_index_t> feature_id_database;

  /**
   * @brief Feature tracking
   */
  feature_tracker_->Track(
      feature_position_database, feature_age_database, frontend_input.frame_);
  // update id list, feature bearing vector.
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
  // Log features after tracking process
  internal_materials.features_after_tracking_ = feature_position_database;


  /**
   * @brief Feature detection
   */

  // Feature detection
  feature_detector_->UpdateDetection(feature_position_database,
                                     feature_age_database,
                                     frontend_input.frame_,
                                     frontend_input.mask_);
  // Update observing id database and feature bearing vector
  for (const auto& [id, pos] : feature_position_database) {
    feature_id_database.insert(id);
    try {
      feature_bearing_database[id] =
          frontend_input.camera_model_ptr_->Unproject(pos);
    } catch (data::ProjectionErrorException& exception) {
      spdlog::warn("Unprojection error: {}", exception.what());
      feature_bearing_database[id] = {0, 0, 1};
    }
  }
  // Log features after detection
  internal_materials.features_after_detection_ = feature_position_database;

  /**
   * @brief KeyFrame Selection and do verification
   */
  auto latest_keyframe_ptr =
      map_database_->GetFrame(map_database_->latest_key_frame_id_).lock();
  if (latest_keyframe_ptr) {
    int32_t num_frames_after_kf =
        (last_frame_->frame_id_ + 1) - latest_keyframe_ptr->frame_id_;
    int32_t num_inuse_features = 0;
    for (const auto& lm_id : feature_id_database) {
      auto lm_ptr = map_database_->GetLandmark(lm_id).lock();
      if (lm_ptr) {
        if (lm_ptr->is_initialized_) {
          num_inuse_features++;
        }
      }
    }
    double inuse_features_rate =
        static_cast<double>(num_inuse_features) /
        static_cast<double>(feature_id_database.size());
    if (((num_frames_after_kf > parameter_.keyframe_min_frames_after_kf_) && (inuse_features_rate < parameter_.keyframe_new_kf_keypoints_threshold_)) ||
        (inuse_features_rate < parameter_.keyframe_new_kf_keypoints_minimum_threshold_)) {
      data::Frame tmp_frame(0,
                            0,
                            true,
                            frontend_input.camera_model_ptr_,
                            feature_id_database,
                            feature_position_database,
                            feature_bearing_database,
                            feature_age_database);

      auto verified_frame =
          feature_verification_->RemoveOutlier(*latest_keyframe_ptr, tmp_frame);
      feature_id_database = verified_frame.observing_feature_id_;
      feature_position_database =
          verified_frame.observing_feature_point_in_device_;
      feature_bearing_database =
          verified_frame.observing_feature_bearing_in_camera_frame_;
      feature_age_database = verified_frame.feature_point_age_;
      // Log features after verification
      internal_materials.features_after_verification_ =
          feature_position_database;

      /**
       * @brief KeyFrame
       */
      auto output_frame = Frame(last_frame_->frame_id_ + 1,
                                frontend_input.timestamp_,
                                true,
                                frontend_input.camera_model_ptr_,
                                feature_id_database,
                                feature_position_database,
                                feature_bearing_database,
                                feature_age_database);
      output_frame.internal_materials_ = internal_materials;
      return output_frame;
    }
  }

  /**
   * @brief Output results
   */
  auto output_frame = Frame(last_frame_->frame_id_ + 1,
                            frontend_input.timestamp_,
                            false,
                            frontend_input.camera_model_ptr_,
                            feature_id_database,
                            feature_position_database,
                            feature_bearing_database,
                            feature_age_database);
  output_frame.internal_materials_ = internal_materials;
  return output_frame;
}
