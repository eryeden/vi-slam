//
// Created by ery on 2020/06/09.
//

#pragma once

#include <basalt/calibration/calibration.hpp>
#include <opencv2/opencv.hpp>

#include "FeatureTrackerBase.hpp"
#include "Frame.hpp"
#include "LSSDOpticalFlow/frame_to_frame_optical_flow.h"
#include "Landmark.hpp"

namespace vslam::feature {

class FeatureTrackerLSSDLucasKanade : public FeatureTrackerBase {
 public:
  class Parameter {
   public:
    Parameter();

    /**
     * @brief Optical flow and Feature detection
     */
    std::string optical_flow_type_;
    int optical_flow_detection_grid_size_;
    double optical_flow_max_recovered_dist_;
    int optical_flow_pattern_;
    int optical_flow_max_iterations_;
    int optical_flow_levels_;
  };

 public:
  FeatureTrackerLSSDLucasKanade(const Parameter& parameter);

  void Track(FeaturePositionDatabase& feature_position,
             FeatureAgeDatabase& feature_age,
             const cv::Mat& current_frame) override;

 private:
  int64_t timestamp_ns_;

  bool is_first_;

  Parameter parameter_;

  /**
   * @brief For LSSD optical flow computaion
   */
  Eigen::aligned_map<basalt::KeypointId, Eigen::AffineCompact2f> transform_;
  std::shared_ptr<basalt::ManagedImagePyr<u_int16_t>> old_pyramid_ptr_,
      pyramid_ptr_;
};

}  // namespace vslam::feature