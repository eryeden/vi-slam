//
// Created by ery on 2020/05/14.
//

#pragma once

#include <opencv2/opencv.hpp>

#include "FeatureDetectorBase.hpp"
#include "Frame.hpp"

namespace vslam::feature {

enum class FeatureDetectorType { FAST, ORB, AGAST, GFTT };

class FeatureDetectorANMS : public FeatureDetectorBase {
 public:
  FeatureDetectorANMS();

  void UpdateDetection(FeaturePositionDatabase& feature_position,
                       FeatureAgeDatabase& feature_age,
                       const cv::Mat& current_image,
                       const cv::Mat& mask_image = cv::Mat()) override;

 private:
};

}  // namespace vslam::feature