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
  // Generate mono channel image
  cv::Mat frame_mono;
  cv::cvtColor(undistorted_current_image, frame_mono, CV_BGR2GRAY);

  if (previous_frame) {
  } else {
    spdlog::info("Empty previous frame.");

    // Feature pointを抽出する
    std::vector<cv::Point2f> points_from_input;
    cv::goodFeaturesToTrack(frame_mono, points_from_input, 1000, 0.01, 0.1);
    std::vector<cv::KeyPoint> current_keypoints;
    cv::KeyPoint::convert(points_from_input, current_keypoints);

    std::set<database_index_t> observing_feature_ids;
    EigenAllocatedUnorderedMap<database_index_t, Vec2_t>
        observing_feature_points_in_device;
    for (size_t idx = 0; idx < current_keypoints.size(); idx++) {
      observing_feature_ids.insert(idx);
      observing_feature_points_in_device[idx] =
          Vec2_t{current_keypoints[idx].pt.x, current_keypoints[idx].pt.y};
    }

    return data::Frame(0,
                       0,
                       false,
                       data::PinholeCameraModel(),
                       observing_feature_ids,
                       observing_feature_points_in_device);
  }
}
