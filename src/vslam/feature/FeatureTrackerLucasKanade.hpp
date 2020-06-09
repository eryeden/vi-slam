//
// Created by ery on 2020/05/05.
//

#pragma once

#include <basalt/calibration/calibration.hpp>
#include <opencv2/opencv.hpp>

#include "FeatureTrackerBase.hpp"
#include "Frame.hpp"
#include "LSSDOpticalFlow/frame_to_frame_optical_flow.h"
#include "Landmark.hpp"

namespace vslam::feature {

class FeatureTrackerLucasKanade : public FeatureTrackerBase {
 public:
  FeatureTrackerLucasKanade(int32_t klt_max_iteration,
                            double klt_epsilon,
                            int32_t klt_window_size,
                            int32_t klt_max_level,
                            double backtrack_distance_threshold);

  void Track(FeaturePositionDatabase& feature_position,
             FeatureAgeDatabase& feature_age,
             const cv::Mat& current_frame) override;

  void Track(FeaturePositionDatabase& feature_position,
             FeatureAgeDatabase& feature_age,
             const cv::Mat& prev_image,
             const cv::Mat& current_frame);

 private:
  void TrackByLucasKanade(FeaturePositionDatabase& feature_position,
                          FeatureAgeDatabase& feature_age,
                          const cv::Mat& prev_image,
                          const cv::Mat& current_frame,
                          const cv::TermCriteria& termination_criteria,
                          const cv::Size2i& tracker_window_size) const;

  cv::TermCriteria termination_criteria_;
  cv::Size2i tracker_window_size_;
  int32_t tracker_max_level_;
  double backtrack_distance_threshold_;

  cv::Mat old_image_;
};

}  // namespace vslam::feature
