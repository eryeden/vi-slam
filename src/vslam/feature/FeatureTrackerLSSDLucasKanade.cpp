//
// Created by ery on 2020/06/08.
//
#include "FeatureTrackerLSSDLucasKanade.hpp"

#include <spdlog/spdlog.h>

#include "LSSDOpticalFlow/frame_to_frame_optical_flow_single_thread.hpp"

using namespace vslam;
using namespace vslam::feature;

FeatureTrackerLSSDLucasKanade::Parameter::Parameter() {
  /**
   * @brief Optical flow and Feature detection
   */
  optical_flow_type_ = "frame_to_frame";
  optical_flow_detection_grid_size_ = 32;
  optical_flow_max_recovered_dist_ = 0.09;
  optical_flow_pattern_ = 51;
  optical_flow_max_iterations_ = 5;
  optical_flow_levels_ = 3;
}

FeatureTrackerLSSDLucasKanade::FeatureTrackerLSSDLucasKanade(
    const Parameter& parameter)
    : FeatureTrackerBase(),
      timestamp_ns_(0),
      is_first_(true),
      parameter_(parameter) {
  old_pyramid_ptr_ = nullptr;
  pyramid_ptr_ = nullptr;
}

void FeatureTrackerLSSDLucasKanade::Track(
    FeaturePositionDatabase& feature_position,
    FeatureAgeDatabase& feature_age,
    const cv::Mat& current_frame) {
  /**
   * @brief convert cv::Mat to basalt::image
   */
  basalt::ManagedImage<uint16_t> input_image(current_frame.cols,
                                             current_frame.rows);
  const uint8_t* data_in = current_frame.ptr();
  uint16_t* data_out = input_image.ptr;
  size_t full_size = current_frame.cols * current_frame.rows;
  for (size_t i = 0; i < full_size; i++) {
    int val = data_in[i];
    val = val << 8;
    data_out[i] = val;
  }
  // Allocate image pyramid
  pyramid_ptr_.reset(new basalt::ManagedImagePyr<u_int16_t>);
  pyramid_ptr_->setFromImage(input_image, parameter_.optical_flow_levels_);

  // 初回
  if (old_pyramid_ptr_ == nullptr) {
    old_pyramid_ptr_ = pyramid_ptr_;

    /// Update keypoint transforms
    for (const auto& [id, pos] : feature_position) {
      Eigen::AffineCompact2f transform;
      transform.setIdentity();
      transform.translation() = pos.cast<float>();
      transform_[id] = transform;
    }

    return;
  } else {
    /// Update keypoint transforms
    //@{
    // Add new newly observed features
    for (const auto& [id, pos] : feature_position) {
      if (transform_.count(id) == 0) {
        Eigen::AffineCompact2f transform;
        transform.setIdentity();
        transform.translation() = pos.cast<float>();
        transform_[id] = transform;
      }
    }
    // Remove deleted landmarks
    std::vector<database_index_t> remove_id;
    for (const auto& [id, tr] : transform_) {
      if (feature_position.count(id) == 0) {
        remove_id.emplace_back(id);
      }
    }
    for (const auto id : remove_id) {
      transform_.erase(id);
    }
    //@}
    Eigen::aligned_map<basalt::KeypointId, Eigen::AffineCompact2f>
        new_transform;
    feature::FrameToFrameOpticalFlow::TrackPoints(
        *old_pyramid_ptr_,
        *pyramid_ptr_,
        transform_,
        new_transform,
        parameter_.optical_flow_max_recovered_dist_,
        parameter_.optical_flow_levels_,
        parameter_.optical_flow_max_iterations_);
    transform_ = new_transform;

    /// Update feature position
    FeaturePositionDatabase new_position;
    FeatureAgeDatabase new_age;
    for (const auto& [id, tr] : transform_) {
      new_position[id] = tr.translation().cast<double>();
      new_age[id] = feature_age[id];
    }

    feature_position = new_position;
    feature_age = new_age;

    old_pyramid_ptr_ = pyramid_ptr_;
  }

  return;
}
