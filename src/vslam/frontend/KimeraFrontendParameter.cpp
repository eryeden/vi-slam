//
// Created by ery on 2020/06/12.
//

#include "KimeraFrontend.hpp"

using namespace vslam;
using namespace vslam::frontend;

KimeraFrontend::Parameter::Parameter() {
  keyframe_interval_ = 5.0;
  minimum_keyframe_interval_ = 0.5;
  low_keyframe_feature_number_ = 200;
  counting_feature_age_ = 2;
}
