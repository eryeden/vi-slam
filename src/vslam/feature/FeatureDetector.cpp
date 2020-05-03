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
  auto image_size = frame_mono.size();

  if (previous_frame) {
  } else {
    spdlog::info("Empty previous frame.");

    // 画像をグリッドで分割して均等な分布になるようにする
    double dx, dy;
    dx = static_cast<double>(image_size.width) /
         static_cast<double>(division_number_col);
    dy = static_cast<double>(image_size.height) /
         static_cast<double>(division_number_row);
    std::vector<cv::Rect2f> grid_def(0);
    grid_def.reserve(division_number_col * division_number_row);
    for (int32_t x = 0; x < division_number_col; x++) {
      for (int32_t y = 0; y < division_number_row; y++) {
        cv::Point2f left_up(x * dx, y * dy);
        cv::Point2f right_down = left_up + cv::Point2f(dx, dy);
        grid_def.emplace_back(cv::Rect2f(left_up, right_down));
      }
    }
    // Gridに分割して特徴点の追加を行う
    int32_t grid_max_feature_number =
        max_feature_number / (division_number_row * division_number_col);

    std::set<database_index_t> observing_feature_ids;
    EigenAllocatedUnorderedMap<database_index_t, Vec2_t>
        observing_feature_points_in_device;
    uint64_t feature_index = 0;
    for (const auto& grect : grid_def) {
      cv::Mat div_image = frame_mono(grect);
      // Feature pointを抽出する
      std::vector<cv::Point2f> points_from_input;
      cv::goodFeaturesToTrack(
          div_image, points_from_input, grid_max_feature_number, 0.01, 0.1);
      std::vector<cv::KeyPoint> current_keypoints;
      cv::KeyPoint::convert(points_from_input, current_keypoints);
      for (const auto& kp : current_keypoints) {
        observing_feature_ids.insert(feature_index);
        observing_feature_points_in_device[feature_index] =
            Vec2_t{kp.pt.x + grect.x, kp.pt.y + grect.y};
        feature_index++;
      }
    }

    return data::Frame(0,
                       0,
                       false,
                       data::PinholeCameraModel(),
                       observing_feature_ids,
                       observing_feature_points_in_device);
  }
}
