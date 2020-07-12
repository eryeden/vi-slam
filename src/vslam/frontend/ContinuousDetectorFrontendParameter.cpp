//
// Created by ery on 2020/06/18.
//
#include "ContinuousDetectorFrontend.hpp"

vslam::frontend::ContinuousDetectorFrontend::Parameter::Parameter() {

  keyframe_min_frames_after_kf_ = 5;
  keyframe_new_kf_keypoints_threshold_ = 0,5;
  keyframe_new_kf_keypoints_minimum_threshold_ = 0.2;

}
