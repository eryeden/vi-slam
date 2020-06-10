//
// Created by ery on 2020/05/14.
//

#include "FeatureDetectorANMS.hpp"

#include <spdlog/spdlog.h>

#define SHOW_DEBUG

namespace vslam::feature {

FeatureDetectorANMS::Parameter::Parameter() {
  /**
   * @brief 最終的に出力するFeatureの設定
   */
  max_feature_number_ = 300;
  min_feature_distance_ = 10.0;

  /**
   * @brief Internal detector
   */
  detection_feature_number_ = max_feature_number_ * 2;
  detection_min_feature_distance_ = 1.0;
  detection_quality_level_ = 0.001;
  detection_block_size_ = 4;
  detection_use_harris_corner_ = false;
  detection_k_ = 0.04;
  detection_ignore_edge_ = 20;

  /**
   * @brief ANMS
   */
  non_max_suppression_tolerance_ = 0.1;  // 0.1;
}

vslam::feature::FeatureDetectorANMS::FeatureDetectorANMS(
    const Parameter parameter)
    : max_feature_index_(0), parameter_(parameter) {
  // Generate feature detector
  feature_detector_ =
      cv::GFTTDetector::create(parameter_.detection_feature_number_,
                               parameter_.detection_quality_level_,
                               parameter_.detection_min_feature_distance_,
                               parameter_.detection_block_size_,
                               parameter_.detection_use_harris_corner_,
                               parameter_.detection_k_);

  // generate ANMS handler
  non_max_suppression_ =
      //      std::make_unique<AdaptiveNonMaximumSuppression>(AnmsAlgorithmType::SDC);
      std::make_unique<AdaptiveNonMaximumSuppression>(AnmsAlgorithmType::Ssc);
}

void vslam::feature::FeatureDetectorANMS::UpdateDetection(
    vslam::FeaturePositionDatabase& feature_position,
    vslam::FeatureAgeDatabase& feature_age,
    const cv::Mat& current_image,
    const cv::Mat& mask_image) {
  Detect(feature_position,
         feature_age,
         current_image,
         parameter_.max_feature_number_,
         parameter_.min_feature_distance_,
         max_feature_index_,
         mask_image);
}

void vslam::feature::FeatureDetectorANMS::Detect(
    vslam::FeaturePositionDatabase& feature_position,
    vslam::FeatureAgeDatabase& feature_age,
    const cv::Mat& current_image,
    int32_t max_feature_number,
    double min_feature_distance,
    vslam::database_index_t& max_feature_index,
    const cv::Mat& mask_image) const {
  /**
   * @brief 設定項目
   * @details
   * 結論：特徴点の追加操作は特徴点密度のキープが目的。特徴点位置の高精度化が目的ではないとする。特徴点位置の精度は、Verificationで確保する想定。
   *   - なので、goodFeaturesToTrackと同じ最低距離とする。
   * - division number
   * - feature point number
   *
   */

  // Input check
  if (feature_position.size() > max_feature_number) {
    spdlog::warn("{}:{} Input feature seems to much. In:{} / Desired:{}",
                 __FILE__,
                 __FUNCTION__,
                 feature_position.size(),
                 max_feature_number);
  }

  // Generate mono channel image
  cv::Mat frame_mono;
  if (current_image.channels() != 1) {
    cv::cvtColor(current_image, frame_mono, CV_BGR2GRAY);
  } else {
    frame_mono = current_image;
  }
  auto image_size = frame_mono.size();

  // 特徴点位置マクス、観測済み位置のマスクを生成する

  cv::Rect edge_roi(parameter_.detection_ignore_edge_,
                    parameter_.detection_ignore_edge_,
                    image_size.width - 2 * parameter_.detection_ignore_edge_,
                    image_size.height - 2 * parameter_.detection_ignore_edge_);
  cv::Mat edge_mask(image_size, CV_8UC1);
  edge_mask = 0;
  cv::rectangle(edge_mask, edge_roi, cv::Scalar(1), CV_FILLED);

  cv::Mat flag_img;  // = cv::Mat::zeros(image_size, CV_8UC1);
  if (mask_image.empty()) {
    flag_img = cv::Mat(image_size, CV_8UC1, 1);
  } else {
    mask_image.copyTo(
        flag_img);  // このカメラマスク上に、観測済み特徴点を、最低距離の半径を持つ円としてマスクしていく
  }
  for (const auto& [id, pos] : feature_position) {
    if (pos[0] > 0 && pos[0] < image_size.width && pos[1] > 0 &&
        pos[1] < image_size.height) {
      cv::circle(flag_img,
                 cv::Point(pos[0], pos[1]),
                 static_cast<int>(min_feature_distance),
                 cv::Scalar(0),
                 cv::FILLED);
    }
  }
  cv::bitwise_and(flag_img, edge_mask, flag_img);

  // Feature pointを抽出する
  std::vector<cv::KeyPoint> detected_keypoints;
  feature_detector_->detect(frame_mono, detected_keypoints, flag_img);
  int32_t n_detected_point = detected_keypoints.size();

  cv::Mat kpvis;
  cv::drawKeypoints(frame_mono, detected_keypoints, kpvis);

  cv::imshow("Mask", flag_img * 255);
  cv::imshow("Kp", kpvis);
  cv::waitKey(1);

  /**
   * @brief Refine Keypoint
   */
  //@{
  //    std::vector<cv::Point2f> detected_points;
  //    for(const auto pk : detected_keypoints){
  //      detected_points.emplace_back(cv::Point2f(pk.pt.x, pk.pt.y));
  //    }
  //    auto criteria = cv::TermCriteria(cv::TermCriteria::COUNT +
  //    cv::TermCriteria::EPS,
  //                                     30,
  //                                     0.1);
  //    cv::cornerSubPix(frame_mono,
  //                     detected_points,
  //                     {5,5},
  //                     {-1,1},
  //                     criteria);
  //    for(size_t i = 0; i < detected_keypoints.size(); i++){
  //      detected_keypoints[i].pt = detected_points[i];
  //    }
  //@}

  /**
   * @brief ANMS
   * @details
   * SSCを利用する場合、必要な特徴点数よりも入力特徴点数が少ないと内部でZero割が発生するので注意。
   */
  //@{
  int32_t need_n_corners = max_feature_number - feature_position.size();
  need_n_corners = (need_n_corners > 0) ? need_n_corners : 0;
  std::vector<cv::KeyPoint>& max_suppressed_keypoints = detected_keypoints;
  if (non_max_suppression_ && (need_n_corners < detected_keypoints.size())) {
    max_suppressed_keypoints = non_max_suppression_->suppressNonMax(
        detected_keypoints,
        need_n_corners,
        static_cast<float>(parameter_.non_max_suppression_tolerance_),
        frame_mono.cols,
        frame_mono.rows);
  } else {
    max_suppressed_keypoints = detected_keypoints;
  }
  //@}
  spdlog::info("sup/det : {}/{} , need : {}",
               max_suppressed_keypoints.size(),
               n_detected_point,
               need_n_corners);

  // Fill database
  FeaturePositionDatabase& observing_feature_points_in_device =
      feature_position;
  FeatureAgeDatabase& feature_point_age = feature_age;

  uint64_t feature_index = max_feature_index + 1;
  for (const auto& kp : max_suppressed_keypoints) {
    observing_feature_points_in_device[feature_index] =
        Vec2_t{kp.pt.x, kp.pt.y};
    feature_point_age[feature_index] = 1;
    feature_index++;
  }
  //  for (const auto& pt : max_suppressed_points) {
  //    observing_feature_points_in_device[feature_index] =
  //        Vec2_t{pt.x, pt.y};
  //    feature_point_age[feature_index] = 1;
  //    feature_index++;
  //  }
  max_feature_index = feature_index - 1;
}

}  // namespace vslam::feature