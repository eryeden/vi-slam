//
// Created by ery on 2020/06/09.
//

#pragma once

#include <basalt/calibration/calibration.hpp>
#include <opencv2/opencv.hpp>

#include "Frame.hpp"
#include "LSSDOpticalFlow/frame_to_frame_optical_flow.h"
#include "Landmark.hpp"

namespace vslam::feature {

class FeatureTrackerBase {
 public:
  FeatureTrackerBase() = default;
  virtual ~FeatureTrackerBase() = default;

  virtual void Track(FeaturePositionDatabase& feature_position,
                     FeatureAgeDatabase& feature_age,
                     const cv::Mat& current_frame) = 0;

 private:
};

}  // namespace vslam::feature
