//
// Created by ery on 2020/05/18.
//

#include "ViewerPrimitives.hpp"

#include <opencv2/core/eigen.hpp>

using namespace vslam::viewer;

vslam::viewer::PointCloudPrimitive::PointCloudPrimitive(
    const std::string& tag_name,
    const vslam::EigenAllocatedVector<vslam::Vec3_t>& points_world_frame,
    const vslam::EigenAllocatedVector<vslam::Vec3_t>& colors)
    : PrimitiveBase(),
      tag_name_(tag_name),
      points_world_frame_(points_world_frame),
      colors_(colors) {}

std::string vslam::viewer::PointCloudPrimitive::GetTag() const {
  return tag_name_;
}

cv::viz::Widget vslam::viewer::PointCloudPrimitive::GetWidget() const {
  std::vector<cv::Point3d> point_cloud;
  point_cloud.reserve(points_world_frame_.size());
  for (const auto& p : points_world_frame_) {
    point_cloud.emplace_back(cv::Point3d(p[0], p[1], p[2]));
  }

  if (colors_.empty()) {
    return cv::viz::WCloud(point_cloud);
  } else {
    std::vector<cv::Scalar> color_cloud;
    color_cloud.reserve(colors_.size());
    for (const auto& c : colors_) {
      color_cloud.emplace_back(cv::Scalar(c[0], c[1], c[2]));
    }
    return cv::viz::WCloud(point_cloud, color_cloud);
  }
}

cv::Affine3d vslam::viewer::PointCloudPrimitive::GetPose() const {
  return cv::Affine3d::Identity();
}

vslam::viewer::PointCloudPrimitive* vslam::viewer::PointCloudPrimitive::Clone()
    const {
  return new PointCloudPrimitive(*this);
}

vslam::viewer::CameraPosePrimitive::CameraPosePrimitive(
    const std::string& tag_name,
    const vslam::Mat33_t& intrinsic,
    const vslam::Vec3_t& position_world_frame,
    const vslam::Quat_t& orientation_world_frame,
    const vslam::Vec3_t& color)
    : PrimitiveBase(),
      tag_name_(tag_name),
      intrinsic_(intrinsic),
      position_world_frame_(position_world_frame),
      orientation_world_frame_(orientation_world_frame),
      color_(color) {}

std::string vslam::viewer::CameraPosePrimitive::GetTag() const {
  return tag_name_;
}

cv::viz::Widget vslam::viewer::CameraPosePrimitive::GetWidget() const {
  cv::Matx33d cv_intrinsic_matrix;
  cv::eigen2cv(intrinsic_, cv_intrinsic_matrix);
  cv::viz::WCameraPosition w_camera_position(
      cv_intrinsic_matrix, 1.0, cv::Scalar(color_[0], color_[1], color_[2]));
  return w_camera_position;
}

cv::Affine3d vslam::viewer::CameraPosePrimitive::GetPose() const {
  cv::Mat tmp_camera_attitude;
  cv::eigen2cv(orientation_world_frame_.toRotationMatrix(),
               tmp_camera_attitude);
  cv::Affine3d tmp_cam_pose(tmp_camera_attitude,
                            cv::Vec3f(position_world_frame_[0],
                                      position_world_frame_[1],
                                      position_world_frame_[2]));
  return tmp_cam_pose;
}

vslam::viewer::CameraPosePrimitive* vslam::viewer::CameraPosePrimitive::Clone()
    const {
  return new CameraPosePrimitive(*this);
}

vslam::viewer::CoordinateSystemPrimitive::CoordinateSystemPrimitive(
    const std::string& tag_name,
    const Vec3_t& position_world_frame,
    const Quat_t& orientation_world_frame)
    : PrimitiveBase(),
      tag_name_(tag_name),
      position_world_frame_(position_world_frame),
      orientation_world_frame_(orientation_world_frame) {}

std::string vslam::viewer::CoordinateSystemPrimitive::GetTag() const {
  return tag_name_;
}
cv::viz::Widget vslam::viewer::CoordinateSystemPrimitive::GetWidget() const {
  return cv::viz::WCoordinateSystem();
}
cv::Affine3d vslam::viewer::CoordinateSystemPrimitive::GetPose() const {
  cv::Mat tmp_camera_attitude;
  cv::eigen2cv(orientation_world_frame_.toRotationMatrix(),
               tmp_camera_attitude);
  cv::Affine3d tmp_cam_pose(tmp_camera_attitude,
                            cv::Vec3f(position_world_frame_[0],
                                      position_world_frame_[1],
                                      position_world_frame_[2]));
  return tmp_cam_pose;
}

vslam::viewer::CoordinateSystemPrimitive*
vslam::viewer::CoordinateSystemPrimitive::Clone() const {
  return new CoordinateSystemPrimitive(*this);
}
