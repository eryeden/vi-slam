//
// Created by ery on 2020/04/30.
//

#include "KimeraFrontend.hpp"

#include "OpenCVUtilities.hpp"

using namespace vslam::data;
using namespace vslam::frontend;

/**
 * @brief Kimera-VIOベースの単眼Frontend
 * @param threadsafe_map_database
 */
KimeraFrontend::KimeraFrontend(
    const std::shared_ptr<data::ThreadsafeMapDatabase>& threadsafe_map_database)
    : FrontendBase(threadsafe_map_database), is_first_frame_(true) {}

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

  return FrontendStatus();
}

/**
 * @brief 一番最初のフレームの処理
 * @param frontend_input
 * @return
 */
Frame KimeraFrontend::ProcessFirstFrame(
    const KimeraFrontendInput& frontend_input) {}

/**
 * @brief 2Frame以降の処理
 * @param frontend_input
 * @param last_frame
 * @param last_keyframe
 * @return
 */
Frame KimeraFrontend::ProcessFrame(const KimeraFrontendInput& frontend_input,
                                   const FrameSharedPtr& last_frame,
                                   const FrameSharedPtr& last_keyframe) {}
