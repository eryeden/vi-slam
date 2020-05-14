//
// Created by ery on 2020/05/14.
//

#pragma once

#include <opencv2/opencv.hpp>

#include "Frame.hpp"

namespace vslam::feature {

class FeatureDetectorBase {
 public:
  FeatureDetectorBase();

  virtual void UpdateDetection(FeaturePositionDatabase& feature_position,
                               FeatureAgeDatabase& feature_age,
                               const cv::Mat& current_image,
                               const cv::Mat& mask_image = cv::Mat()) = 0;
};

}  // namespace vslam::feature
