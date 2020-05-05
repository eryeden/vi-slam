//
// Created by ery on 2020/05/05.
//
#include "FeatureTrackerLucasKanade.hpp"

#include <spdlog/spdlog.h>

vslam::feature::FeatureTrackerLucasKanade::FeatureTrackerLucasKanade(
    int32_t klt_max_iteration,
    double klt_epsilon,
    int32_t klt_window_size,
    int32_t klt_max_level)
    : termination_criteria_(
          cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS,
                           klt_max_iteration,
                           klt_epsilon)),
      tracker_window_size_(cv::Size2i(klt_window_size, klt_window_size)),
      tracker_max_level_(klt_max_level) {
  ;
}

void vslam::feature::FeatureTrackerLucasKanade::Track(
    vslam::FeaturePositionDatabase& feature_position,
    vslam::FeatureAgeDatabase& feature_age,
    const cv::Mat& prev_image,
    const cv::Mat& current_frame) {
  TrackByLucasKanade(feature_position,
                     feature_age,
                     prev_image,
                     current_frame,
                     termination_criteria_,
                     tracker_window_size_);
}

void vslam::feature::FeatureTrackerLucasKanade::TrackByLucasKanade(
    vslam::FeaturePositionDatabase& feature_position,
    vslam::FeatureAgeDatabase& feature_age,
    const cv::Mat& prev_image,
    const cv::Mat& current_frame,
    const cv::TermCriteria& termination_criteria,
    const cv::Size2i& tracker_window_size) const {
  // Check inputs
  if (feature_position.empty() || feature_age.empty()) {
    spdlog::warn("{}:{} Empty inputs.", __FILE__, __FUNCTION__);
    return;
  }

  std::vector<database_index_t> feature_id;
  std::vector<cv::Point2f> prev_point;
  std::vector<cv::Point2f> estimated_point;

  // feed id and position
  feature_id.reserve(feature_position.size());
  prev_point.reserve(feature_position.size());
  estimated_point.reserve(feature_position.size());
  for (const auto& [id, pos] : feature_position) {
    feature_id.emplace_back(id);
    prev_point.emplace_back(cv::Point2f(pos[0], pos[1]));
  }

  // calculate flow
  std::vector<uint8_t> tracker_status;
  std::vector<float> tracking_error;
  cv::calcOpticalFlowPyrLK(prev_image,
                           current_frame,
                           prev_point,
                           estimated_point,
                           tracker_status,
                           tracking_error,
                           tracker_window_size,
                           tracker_max_level_,
                           termination_criteria);

  // output
  FeaturePositionDatabase estimated_feature_position;
  FeatureAgeDatabase new_feature_age;
  for (size_t idx = 0; idx < estimated_point.size(); idx++) {
    if (tracker_status[idx] == 1) {
      estimated_feature_position[feature_id[idx]] = {estimated_point[idx].x,
                                                     estimated_point[idx].y};
      new_feature_age[feature_id[idx]] = feature_age[feature_id[idx]];
    }
  }

  // overwrite inputs
  feature_position = estimated_feature_position;
  feature_age = new_feature_age;
}
