//
// Created by ery on 2020/05/03.
//

#include "FeatureDetector.hpp"

#include <spdlog/spdlog.h>

vslam::data::Frame vslam::feature::DetectShiTomasiCorners(
    const vslam::data::FrameSharedPtr& previous_frame,
    const cv::Mat& undistorted_current_image,
    int32_t division_number_row,
    int32_t division_number_col,
    int32_t max_feature_number) {
  if (previous_frame) {
  } else {
    spdlog::info("Empty previous frame.");
  }
}
