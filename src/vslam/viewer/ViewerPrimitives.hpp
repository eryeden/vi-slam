//
// Created by ery on 2020/05/18.
//

#pragma once

#include <opencv2/opencv.hpp>
#include <opencv2/viz.hpp>
#include <opencv2/viz/types.hpp>
#include <opencv2/viz/widgets.hpp>
#include <string>

#include "type_defines.hpp"

namespace vslam::viewer {

class PrimitiveBase {
 public:
  PrimitiveBase() = default;
  virtual ~PrimitiveBase() = default;

  virtual std::string GetTag() const = 0;
  virtual cv::viz::Widget GetWidget() const = 0;
  virtual cv::Affine3d GetPose() const = 0;

  virtual PrimitiveBase* Clone() const = 0;

 private:
};

class PointCloudPrimitive : public PrimitiveBase {
 public:
  /**
   * @brief
   * @param tag_name
   * @param points_world_frame
   * @param colors
   */
  PointCloudPrimitive(
      const std::string& tag_name,
      const vslam::EigenAllocatedVector<vslam::Vec3_t>& points_world_frame,
      const vslam::EigenAllocatedVector<vslam::Vec3_t>& colors =
          vslam::EigenAllocatedVector<vslam::Vec3_t>());

  std::string GetTag() const override;
  cv::viz::Widget GetWidget() const override;
  cv::Affine3d GetPose() const override;
  PointCloudPrimitive* Clone() const override;

 private:
  const std::string tag_name_;
  const vslam::EigenAllocatedVector<vslam::Vec3_t> points_world_frame_;
  const vslam::EigenAllocatedVector<vslam::Vec3_t> colors_;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class CovariancePrimitive : public PrimitiveBase {
 public:
  CovariancePrimitive(const std::string& tag_name,
                      const vslam::Vec3_t& position_world_frame,
                      const vslam::Quat_t& orientation_world_T_current,
                      const vslam::Mat33_t& covariance_current_frame,
                      const vslam::Vec3_t& color = vslam::Vec3_t(),
                      double chi_chi = 9.21034);

  std::string GetTag() const override;
  cv::viz::Widget GetWidget() const override;
  cv::Affine3d GetPose() const override;
  CovariancePrimitive* Clone() const override;

 private:
  const std::string tag_name_;
  const vslam::Vec3_t position_world_frame_;
  const vslam::Quat_t orientation_world_T_current_;
  const vslam::Mat33_t covariance_current_frame_;
  const vslam::Vec3_t color_;

  vslam::Vec3_t ellipsoid_scale_;
  vslam::Mat33_t rotation_current_T_ellipsoid_;
  const double chi_chi_;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class CameraPosePrimitive : public PrimitiveBase {
 public:
  CameraPosePrimitive(const std::string& tag_name,
                      const vslam::Mat33_t& intrinsic,
                      const vslam::Vec3_t& position_world_frame,
                      const vslam::Quat_t& orientation_world_frame,
                      const vslam::Vec3_t& color);

  std::string GetTag() const override;
  cv::viz::Widget GetWidget() const override;
  cv::Affine3d GetPose() const override;
  CameraPosePrimitive* Clone() const override;

 private:
  const std::string tag_name_;
  const vslam::Mat33_t intrinsic_;
  const vslam::Vec3_t position_world_frame_;
  const vslam::Quat_t orientation_world_frame_;
  const vslam::Vec3_t color_;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class CoordinateSystemPrimitive : public PrimitiveBase {
 public:
  CoordinateSystemPrimitive(const std::string& tag_name,
                            const vslam::Vec3_t& position_world_frame,
                            const vslam::Quat_t& orientation_world_frame);

  std::string GetTag() const override;
  cv::viz::Widget GetWidget() const override;
  cv::Affine3d GetPose() const override;
  CoordinateSystemPrimitive* Clone() const override;

 private:
  const std::string tag_name_;
  const vslam::Vec3_t position_world_frame_;
  const vslam::Quat_t orientation_world_frame_;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class QuadricPrimitive : public PrimitiveBase {
 public:
  QuadricPrimitive(const std::string& tag_name,
                   const vslam::Vec3_t& quadric_scale,
                   const vslam::Vec3_t& position_world_frame,
                   const vslam::Quat_t& orientation_world_frame,
                   const vslam::Vec3_t& color = vslam::Vec3_t());

  std::string GetTag() const override;
  cv::viz::Widget GetWidget() const override;
  cv::Affine3d GetPose() const override;
  QuadricPrimitive* Clone() const override;

 private:
  const std::string tag_name_;
  const vslam::Vec3_t quadric_scale_;
  const vslam::Vec3_t position_world_frame_;
  const vslam::Quat_t orientation_world_frame_;
  const vslam::Vec3_t color_;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace vslam::viewer