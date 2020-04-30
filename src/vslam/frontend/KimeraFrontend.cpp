//
// Created by ery on 2020/04/30.
//

#include "KimeraFrontend.hpp"

using namespace vslam::data;
using namespace vslam::frontend;

/**
 * @brief Kimera-VIOベースの単眼Frontend
 * @param threadsafe_map_database
 */
KimeraFrontend::KimeraFrontend(
    const std::shared_ptr<data::ThreadsafeMapDatabase>& threadsafe_map_database)
    : FrontendBase(threadsafe_map_database) {}

/**
 * @brief Implementation of Kimera-based Vision-Frontend
 * @details
 * Frontendのメインループとなるので、ここを起点にFrontendの全処理が記述される
 * @param image
 * @return
 */
FrontendStatus KimeraFrontend::FeedImage(const cv::Mat& image) {
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
   */

  return FrontendStatus();
}
