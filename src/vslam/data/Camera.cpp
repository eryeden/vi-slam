//
// Created by anudev on 2020/02/10.
//

#include "Camera.hpp"

#include "spdlog/spdlog.h"

using namespace vslam::data;

vslam::data::CameraModelBase::CameraModelBase(uint8_t camera_id,
                                              uint32_t image_width_pixel,
                                              uint32_t image_height_pixel,
                                              double rate)
    : id_(camera_id),
      width_(image_width_pixel),
      height_(image_height_pixel),
      rate_(rate) {}

vslam::data::CameraModelBase::CameraModelBase() : CameraModelBase(0, 0, 0, 0) {}

vslam::data::RadialTangentialCameraModel::RadialTangentialCameraModel(
    uint8_t id_,
    uint32_t width_,
    uint32_t height_,
    double fps_,
    double fx_,
    double fy_,
    double cx_,
    double cy_,
    double k1_,
    double k2_,
    double p1_,
    double p2_,
                                                    double k3_)
    : id(id_),
      width(width_),
      height(height_),
      fps(fps_),
      fx(fx_),
      fy(fy_),
      cx(cx_),
      cy(cy_),
      k1(k1_),
      k2(k2_),
      p1(p1_),
      p2(p2_),
      k3(k3_),
      fx_inv(1.0 / fx_),
      fy_inv(1.0 / fy_) {
  ;
}

vslam::data::RadialTangentialCameraModel::RadialTangentialCameraModel()
    : RadialTangentialCameraModel(std::numeric_limits<uint8_t>::max(),
                                  0,
                                  0,
                                  0,
                                  0,
                                  0,
                                  0,
                                  0,
                                  0,
                                  0,
                                  0,
                                  0,
                         0) {
  ;
}

vslam::Mat33_t vslam::data::RadialTangentialCameraModel::GetIntrinsicMatrix()
    const {
  Mat33_t out;
  out << fx, 0, cx, 0, fy, cy, 0, 0, 1.0;
  return out;
}
std::vector<double>
vslam::data::RadialTangentialCameraModel::GetDistortionParameters() const {
  return {k1, k2, p1, p2, k3};
}

void vslam::data::RadialTangentialCameraModel::SetCameraIntrinsicParameter(
    const vslam::Mat33_t& intrinsic) {
  fx = intrinsic(0, 0);
  fy = intrinsic(1, 1);
  cx = intrinsic(2, 0);
  cy = intrinsic(2, 1);
  fx_inv = 1.0 / fx;
  fy_inv = 1.0 / fy;
}

void vslam::data::RadialTangentialCameraModel::SetCameraDistortionParameter(
    const std::vector<double>& distortion_parameter) {
  k1 = distortion_parameter[0];
  k2 = distortion_parameter[1];
  p1 = distortion_parameter[2];
  p2 = distortion_parameter[3];
  k3 = distortion_parameter[4];
}

DoubleSphereCameraModel::DoubleSphereCameraModel(uint8_t camera_id,
                                                 uint32_t image_width_pixel,
                                                 uint32_t image_height_pixel,
                                                 double rate,
                                                 double fx,
                                                 double fy,
                                                 double cx,
                                                 double cy,
                                                 double xi,
                                                 double alpha)
    : CameraModelBase(camera_id, image_width_pixel, image_height_pixel, rate),
      fx_(fx),
      fy_(fy),
      cx_(cx),
      cy_(cy),
      xi_(xi),
      alpha_(alpha) {
  Eigen::Matrix<double, 6, 1> ds_parameters;
  ds_parameters << fx_, fy_, cx_, cy_, xi_, alpha_;
  ds_camera_model_ptr_ =
      std::make_unique<basalt::DoubleSphereCamera<double>>(ds_parameters);
}
DoubleSphereCameraModel::DoubleSphereCameraModel()
    : DoubleSphereCameraModel(0, 0, 0, 0, 0, 0, 0, 0, 0, 0) {
  ;
}

DoubleSphereCameraModel::DoubleSphereCameraModel(
    const DoubleSphereCameraModel& double_sphere_camera_model)
    : fx_(double_sphere_camera_model.fx_),
      fy_(double_sphere_camera_model.fy_),
      cx_(double_sphere_camera_model.cx_),
      cy_(double_sphere_camera_model.cy_),
      xi_(double_sphere_camera_model.xi_),
      alpha_(double_sphere_camera_model.alpha_) {
  Eigen::Matrix<double, 6, 1> ds_parameters;
  ds_parameters << fx_, fy_, cx_, cy_, xi_, alpha_;
  ds_camera_model_ptr_ =
      std::make_unique<basalt::DoubleSphereCamera<double>>(ds_parameters);
}

DoubleSphereCameraModel* DoubleSphereCameraModel::Clone() {
  return new DoubleSphereCameraModel(*this);
}

vslam::Vec2_t DoubleSphereCameraModel::Project(
    const vslam::Vec3_t& pos_camera_frame) const {
  Vec2_t pos_image_frame;
  if (ds_camera_model_ptr_->project(
          {pos_camera_frame[0], pos_camera_frame[1], pos_camera_frame[2], 0},
          pos_image_frame)) {
    spdlog::warn("{}:{} Invalid Projection.", __FILE__, __FUNCTION__);
  }
  return pos_image_frame;
}

vslam::Vec3_t DoubleSphereCameraModel::Unproject(
    const vslam::Vec2_t& pos_image_frame) const {
  Vec4_t pos_camera_frame;
  if (ds_camera_model_ptr_->unproject(pos_image_frame, pos_camera_frame)) {
    //    spdlog::warn("{}:{} Invalid Unprojection.", __FILE__, __FUNCTION__);
  }
  return {pos_camera_frame[0], pos_camera_frame[1], pos_camera_frame[2]};
}

vslam::Mat33_t DoubleSphereCameraModel::IntrinsicMatrix() const {
  Mat33_t out;
  out << fx_, 0, cx_, 0, fy_, cy_, 0, 0, 1.0;
  return out;
}
