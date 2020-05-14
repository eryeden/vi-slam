//
// Created by ery on 2020/05/14.
//

#include "FeatureDetectorANMS.hpp"

#include <spdlog/spdlog.h>

#define SHOW_DEBUG

namespace vslam::feature {

vslam::feature::FeatureDetectorANMS::FeatureDetectorANMS(
    int32_t max_feature_number,
    double min_feature_distance)
    : max_feature_index_(0),
      max_feature_number_(max_feature_number),
      min_feature_distance_(min_feature_distance) {
  ;

  // Feature detector params
  int32_t detector_max_feature_number = max_feature_number_ * 10;
  double detector_quality_level = 0.01;
  double detector_min_distance = min_feature_distance_;
  int32_t detector_block_size = 3;
  bool detector_use_harris_corner = false;
  double detector_k = 0.04;

  // Generate feature detector
  feature_detector_ = cv::GFTTDetector::create(detector_max_feature_number,
                                               detector_quality_level,
                                               detector_min_distance,
                                               detector_block_size,
                                               detector_use_harris_corner,
                                               detector_k);

  // generate ANMS handler
  non_max_suppression_ =
      std::make_unique<AdaptiveNonMaximumSuppression>(AnmsAlgorithmType::SDC);
}

void vslam::feature::FeatureDetectorANMS::UpdateDetection(
    vslam::FeaturePositionDatabase& feature_position,
    vslam::FeatureAgeDatabase& feature_age,
    const cv::Mat& current_image,
    const cv::Mat& mask_image) {
  Detect(feature_position,
         feature_age,
         current_image,
         max_feature_number_,
         min_feature_distance_,
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
                 static_cast<int>(min_feature_distance_),
                 cv::Scalar(0),
                 cv::FILLED);
    }
  }

  // Feature pointを抽出する
  std::vector<cv::KeyPoint> detected_keypoints;
  feature_detector_->detect(frame_mono, detected_keypoints, flag_img);
  int32_t n_detected_point = detected_keypoints.size();

  cv::Mat kpvis;
  cv::drawKeypoints(frame_mono, detected_keypoints, kpvis);

  cv::imshow("Mask", flag_img * 255);
  cv::imshow("Kp", kpvis);
  cv::waitKey(1);

  // ANMS
  int32_t need_n_corners = max_feature_number - feature_position.size();
  need_n_corners = (need_n_corners > 0) ? need_n_corners : 0;
  std::vector<cv::KeyPoint>& max_suppressed_keypoints = detected_keypoints;
  if (non_max_suppression_) {
    static constexpr float tolerance = 0.1;
    //      static constexpr float tolerance = 0.001;
    max_suppressed_keypoints =
        non_max_suppression_->suppressNonMax(detected_keypoints,
                                             need_n_corners,
                                             tolerance,
                                             frame_mono.cols,
                                             frame_mono.rows);
  }
  spdlog::info("sup/det : {}/{} , need : {}",
               max_suppressed_keypoints.size(),
               n_detected_point,
               need_n_corners);

  // Fill database
  FeaturePositionDatabase& observing_feature_points_in_device =
      feature_position;
  FeatureAgeDatabase& feature_point_age = feature_age;

  uint64_t feature_index = max_feature_index + 1;
  // 最近傍特徴点からの距離がmin_feature_distance以上かチェック
  for (const auto& kp : max_suppressed_keypoints) {
    observing_feature_points_in_device[feature_index] =
        Vec2_t{kp.pt.x, kp.pt.y};
    feature_point_age[feature_index] = 1;
    feature_index++;
  }
  max_feature_index = feature_index - 1;
}

}  // namespace vslam::feature