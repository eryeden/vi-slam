//
// Created by ery on 2020/04/30.
//

#pragma once

#include "FeatureDetectorBase.hpp"
#include "FeatureTrackerBase.hpp"
#include "FrontendBase.hpp"
#include "Verification.hpp"

namespace vslam::frontend {

class KimeraFrontendInputRadialTangentialCameraModel {
 public:
  double timestamp_;
  cv::Mat frame_;
  data::RadialTangentialCameraModel camera_model_;

 private:
};

class KimeraFrontendInput {
 public:
  KimeraFrontendInput();
  KimeraFrontendInput(
      double timestamp,
      const cv::Mat& frame,
      const cv::Mat& mask,
      const std::unique_ptr<data::CameraModelBase>& camera_model);
  KimeraFrontendInput(const KimeraFrontendInput& kimera_frontend_input);

  KimeraFrontendInput& operator=(
      const KimeraFrontendInput& kimera_frontend_input);

  double timestamp_;
  cv::Mat frame_;
  cv::Mat mask_;
  std::unique_ptr<data::CameraModelBase> camera_model_ptr_;

 private:
};

class KimeraFrontend : public FrontendBase {
 public:
  class Parameter {
   public:
    Parameter();

    double keyframe_interval_;
    double minimum_keyframe_interval_;
    uint32_t low_keyframe_feature_number_;
    int32_t counting_feature_age_;
  };

 public:
  KimeraFrontend(
      const std::shared_ptr<data::ThreadsafeMapDatabase>&
          threadsafe_map_database,
      const std::shared_ptr<feature::FeatureDetectorBase>& feature_detector,
      const std::shared_ptr<feature::FeatureTrackerBase>& feature_tracker,
      const std::shared_ptr<verification::FeatureVerification5PointRANSAC>&
          feature_verification,
      Parameter parameter);

  FrontendStatus Feed(const KimeraFrontendInput& frontend_input);

  // Last input
  KimeraFrontendInput last_input_;

 private:
  data::Frame ProcessFirstFrame(const KimeraFrontendInput& frontend_input);
  data::Frame ProcessFrame(const KimeraFrontendInput& frontend_input,
                           const data::FrameSharedPtr& last_frame,
                           const data::FrameSharedPtr& last_keyframe);
  /**
   * @note
   * 自分用メモ:SharedPrtは上書きでも参照回数がデクリメントされるので、
   * メモリが開放される。　https://beatsync.net/bayside/memo/log20050905.html
   *
   */
  data::FrameSharedPtr last_frame_;
  data::FrameSharedPtr last_keyframe_;

  bool is_first_frame_;

  // Feature Detector
  std::shared_ptr<feature::FeatureDetectorBase> feature_detector_;
  // Feature Tracker
  std::shared_ptr<feature::FeatureTrackerBase> feature_tracker_;
  // Feature Verification
  std::shared_ptr<verification::FeatureVerification5PointRANSAC>
      feature_verification_;

  // Parameter
  Parameter parameter_;
};

}  // namespace vslam::frontend