//
// Created by ery on 2020/04/30.
//

#pragma once

#include "FeatureDetectorShiTomasi.hpp"
#include "FeatureTrackerLucasKanade.hpp"
#include "FrontendBase.hpp"

namespace vslam::frontend {

class KimeraFrontendInput {
 public:
  double timestamp_;
  cv::Mat frame_;
  data::PinholeCameraModel camera_model_;

 private:
};

class KimeraFrontend : public FrontendBase {
 public:
  KimeraFrontend(const std::shared_ptr<data::ThreadsafeMapDatabase>&
                     threadsafe_map_database,
                 const std::shared_ptr<feature::FeatureDetectorShiTomasi>&
                     feature_detector_shi_tomasi,
                 const std::shared_ptr<feature::FeatureTrackerLucasKanade>&
                     feature_tracker_lucas_kanade,
                 double keyframe_interval_threshold,
                 uint32_t keyframe_feature_number_threshold);

  FrontendStatus Feed(const KimeraFrontendInput& frontend_input);

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

  // Last input
  KimeraFrontendInput last_input_;
  // Feature Detector
  std::shared_ptr<feature::FeatureDetectorShiTomasi>
      feature_detector_shi_tomasi_;
  // Feature Tracker
  std::shared_ptr<feature::FeatureTrackerLucasKanade>
      feature_tracker_lucas_kanade_;

  // Parameter
  double keyframe_interval_threshold_;
  uint32_t keyframe_feature_number_threshold_;
};

}  // namespace vslam::frontend