//
// Created by anudev on 2020/02/10.
//

#pragma once

#include <basalt/camera/double_sphere_camera.hpp>
#include <basalt/camera/pinhole_camera.hpp>
#include <exception>
#include <memory>

#include "type_defines.hpp"

namespace vslam::data {

class ProjectionErrorException : public std::exception {
 public:
  ProjectionErrorException();
  const char* what() const throw();

 private:
};

class CameraModelBase {
 public:
  CameraModelBase(uint8_t camera_id,
                  uint32_t image_width_pixel,
                  uint32_t image_height_pixel,
                  double rate);
  CameraModelBase();  //:CameraModelBase(0,0,0,0){}

  virtual ~CameraModelBase() = default;

  /**
   * @brief Deep copy 用
   * @return
   */
  virtual CameraModelBase* Clone() = 0;

  /**
   * @brief カメラ座標系での３次元ポイントを画像上に投影する
   * @param pos_camera_frame
   * @return
   */
  virtual Vec2_t Project(const Vec3_t& pos_camera_frame) const = 0;
  /**
   * カメラ座標系での3次元ポイントを画像座標系上に投影するとともに、3次元ポイントに関するヤコビアンを計算する
   * @param pos_camera_frame
   * @param jacobian_p3d
   * @return
   */
  virtual Vec2_t Project(const Vec3_t& pos_camera_frame,
                         vslam::MatRC_t<2, 3>& jacobian_p3d) const = 0;

  /**
   * @brief 画像上の２Dポイントをカメラ座標系でのBearing vectorに変換する
   * @param pos_image_frame
   * @return
   */
  virtual Vec3_t Unproject(const Vec2_t& pos_image_frame) const = 0;

  /**
   * @brief [fx,0,ux; 0,fy,uy; 0,0,1]からなる行列を返す
   * @return
   */
  virtual Mat33_t IntrinsicMatrix() const = 0;

  //! Camera id
  uint8_t id_;

  //! Width of image
  uint32_t width_;
  //! Height of image
  uint32_t height_;

  //! Frame rate of image
  double rate_;

 private:
};

class RadialTangentialCameraModel {
 public:
  RadialTangentialCameraModel(uint8_t id,
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
  RadialTangentialCameraModel();

  Mat33_t GetIntrinsicMatrix() const;
  std::vector<double> GetDistortionParameters() const;

  void SetCameraIntrinsicParameter(const Mat33_t& intrinsic);
  void SetCameraDistortionParameter(
      const std::vector<double>& distortion_parameter);

  //! RadialTangentialCameraModel id
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

class DoubleSphereCameraModel : public CameraModelBase {
 public:
  /**
   * @brief DoubleSphere camera modelベースのカメラモデル
   * @details
   * カメラパラメータについて詳細は [The Double Sphere Camera
   * Model](https://arxiv.org/abs/1807.08957) 参照。
   * @param camera_id
   * @param image_width_pixel
   * @param image_height_pixel
   * @param rate
   * @param fx
   * @param fy
   * @param cx
   * @param cy
   * @param xi
   * @param alpha
   */
  DoubleSphereCameraModel(uint8_t camera_id,
                          uint32_t image_width_pixel,
                          uint32_t image_height_pixel,
                          double rate,
                          double fx,
                          double fy,
                          double cx,
                          double cy,
                          double xi,
                          double alpha);
  DoubleSphereCameraModel();

  DoubleSphereCameraModel(
      const DoubleSphereCameraModel& double_sphere_camera_model);

  ~DoubleSphereCameraModel() = default;

  /**
   * @brief For deep copy
   * @return
   */
  DoubleSphereCameraModel* Clone();

  /**
   * @brief カメラ座標系での３次元ポイントを画像上に投影する
   * @param pos_camera_frame
   * @return
   */
  Vec2_t Project(const Vec3_t& pos_camera_frame) const;

  /**
   * カメラ座標系での3次元ポイントを画像座標系上に投影するとともに、3次元ポイントに関するヤコビアンを計算する
   * @param pos_camera_frame
   * @param jacobian_p3d
   * @return
   */
  Vec2_t Project(const Vec3_t& pos_camera_frame,
                 vslam::MatRC_t<2, 3>& jacobian_p3d) const;

  /**
   * @brief 画像上の２Dポイントをカメラ座標系でのBearing vectorに変換する
   * @param pos_image_frame
   * @return
   */
  Vec3_t Unproject(const Vec2_t& pos_image_frame) const;

  /**
   * @brief [fx,0,ux; 0,fy,uy; 0,0,1]からなる行列を返す
   * @return
   */
  Mat33_t IntrinsicMatrix() const;

  double fx_;
  double fy_;
  double cx_;
  double cy_;
  double xi_;
  double alpha_;

 private:
  std::unique_ptr<basalt::DoubleSphereCamera<double>> ds_camera_model_ptr_;
};

class PinholeCameraModel : public CameraModelBase {
 public:
  PinholeCameraModel(uint8_t camera_id,
                     uint32_t image_width_pixel,
                     uint32_t image_height_pixel,
                     double rate,
                     double fx,
                     double fy,
                     double cx,
                     double cy);
  PinholeCameraModel();
  PinholeCameraModel(const PinholeCameraModel& pinhole_camera_model);

  ~PinholeCameraModel() = default;

  /**
   * @brief For deep copy
   * @return
   */
  PinholeCameraModel* Clone();

  /**
   * @brief カメラ座標系での３次元ポイントを画像上に投影する
   * @param pos_camera_frame
   * @return
   */
  Vec2_t Project(const Vec3_t& pos_camera_frame) const;

  /**
   * カメラ座標系での3次元ポイントを画像座標系上に投影するとともに、3次元ポイントに関するヤコビアンを計算する
   * @param pos_camera_frame
   * @param jacobian_p3d
   * @return
   */
  Vec2_t Project(const Vec3_t& pos_camera_frame,
                 vslam::MatRC_t<2, 3>& jacobian_p3d) const;

  /**
   * @brief 画像上の２Dポイントをカメラ座標系でのBearing vectorに変換する
   * @param pos_image_frame
   * @return
   */
  Vec3_t Unproject(const Vec2_t& pos_image_frame) const;

  /**
   * @brief [fx,0,ux; 0,fy,uy; 0,0,1]からなる行列を返す
   * @return
   */
  Mat33_t IntrinsicMatrix() const;

  double fx_;
  double fy_;
  double cx_;
  double cy_;

 private:
  std::unique_ptr<basalt::PinholeCamera<double>> pinhole_camera_model_ptr_;
};

}  // namespace vslam::data