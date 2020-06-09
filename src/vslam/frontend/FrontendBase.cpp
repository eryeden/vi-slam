//
// Created by ery on 2020/04/24.
//

#include "FrontendBase.hpp"

using namespace vslam;
using namespace vslam::data;
using namespace vslam::frontend;

FrontendInput::FrontendInput()
    : FrontendInput(0,
                    cv::Mat(),
                    cv::Mat(),
                    std::unique_ptr<CameraModelBase>()) {
  ;
}
FrontendInput::FrontendInput(
    double timestamp,
    const cv::Mat& frame,
    const cv::Mat& mask,
    const std::unique_ptr<data::CameraModelBase>& camera_model)
    : timestamp_(timestamp) {
  frame_ = frame;
  mask_ = mask;
  if (camera_model) {
    camera_model_ptr_ =
        std::unique_ptr<data::CameraModelBase>(camera_model->Clone());
  }
}

FrontendInput::FrontendInput(const FrontendInput& frontend_input)
    : FrontendInput(frontend_input.timestamp_,
                    frontend_input.frame_,
                    frontend_input.mask_,
                    frontend_input.camera_model_ptr_) {}

FrontendInput& FrontendInput::operator=(const FrontendInput& frontend_input) {
  timestamp_ = frontend_input.timestamp_;
  frame_ = frontend_input.frame_;
  mask_ = frontend_input.mask_;
  if (frontend_input.camera_model_ptr_ != nullptr) {
    camera_model_ptr_.reset(frontend_input.camera_model_ptr_->Clone());
  }
  return *this;
}

vslam::frontend::FrontendBase::FrontendBase(
    const std::shared_ptr<data::ThreadsafeMapDatabase>& map_database)
    : map_database_(map_database) {
  ;
}
