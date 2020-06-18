//
// Created by ery on 2020/06/18.
//

#pragma once

#include "FeatureDetectorBase.hpp"
#include "FeatureTrackerBase.hpp"
#include "FrontendBase.hpp"
#include "KimeraFrontend.hpp"
#include "Verification.hpp"

namespace vslam::frontend {

/**
 * @brief 毎Frame Feature tracking -> Feature detection -> Feature
 * Verificationの処理を実行する。
 * @details したがって、KeyFrameの判断はFrontendでは行わない。
 */
class ContinuousDetectorFrontend : public FrontendBase {
 public:
  class Parameter {
   public:
    Parameter();
  };

 public:
  ContinuousDetectorFrontend(
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
  /**
   * @brief 初期Frameの処理。Feature detectionを実行するのみ。
   * @param frontend_input
   * @return
   */
  data::Frame ProcessFirstFrame(const KimeraFrontendInput& frontend_input);

  /**
   * @brief 2 Frame以降の処理。Feature tracking -> Feature detection -> Feature
   * Verificationを毎フレーム実行する。
   * @param frontend_input
   * @param last_frame
   * @return
   */
  data::Frame ProcessFrame(const KimeraFrontendInput& frontend_input,
                           const data::FrameSharedPtr& last_frame);

  /**
   * @brief 前回Frameと前回KeyFrameは頻繁に参照されるので保持しておく。
   */
  data::FrameSharedPtr last_frame_;

  /**
   * @brief 初回処理かどうかに判別に利用
   */
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