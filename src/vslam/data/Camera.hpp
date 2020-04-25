//
// Created by anudev on 2020/02/10.
//

#pragma once

#include "type_defines.hpp"

namespace vslam::data {

class CameraModelBase {
 public:
 private:
};

class PinholeCameraModel : public CameraModelBase {
 public:
  PinholeCameraModel(uint8_t id,
                     uint32_t width,
                     uint32_t height,
                     double fps,
                     double fx,
                     double fy,
                     double cx,
                     double cy,
                     double k1,
                     double k2,
                     double p1,
                     double p2,
                     double k3);
  PinholeCameraModel();

  Mat33_t GetIntrinsicMatrix() const;
  std::vector<double> GetDistortionParameters() const;

  void SetCameraIntrinsicParameter(const Mat33_t& intrinsic);
  void SetCameraDistortionParameter(
      const std::vector<double>& distortion_parameter);

  //! PinholeCameraModel id
  uint8_t id;

  //! width of image
  uint32_t width;
  //! height of image
  uint32_t height;

  //! Frame rate of image
  double fps;

  //! pinhole params
  double fx;
  double fy;
  double cx;
  double cy;
  double fx_inv;
  double fy_inv;

  //! distortion params
  double k1;
  double k2;
  double p1;
  double p2;
  double k3;

 private:
};

}  // namespace vslam::data