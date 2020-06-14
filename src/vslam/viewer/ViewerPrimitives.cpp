//
// Created by ery on 2020/05/18.
//

#include "ViewerPrimitives.hpp"

#include <spdlog/spdlog.h>

#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>

#include "Widget/Quadric.hpp"

using namespace vslam::viewer;

vslam::viewer::PointCloudPrimitive::PointCloudPrimitive(
    const std::string& tag_name,
    const vslam::EigenAllocatedVector<vslam::Vec3_t>& points_world_frame,
    bool draw_point_id,
    const vslam::EigenAllocatedVector<vslam::Vec3_t>& colors)
    : PrimitiveBase(),
      tag_name_(tag_name),
      points_world_frame_(points_world_frame),
      colors_(colors),
      draw_point_id_(draw_point_id) {}

std::string vslam::viewer::PointCloudPrimitive::GetTag() const {
  return tag_name_;
}

std::vector<cv::viz::Widget> vslam::viewer::PointCloudPrimitive::GetWidget()
    const {
  std::vector<cv::Point3d> point_cloud;
  point_cloud.reserve(points_world_frame_.size());
  for (const auto& p : points_world_frame_) {
    point_cloud.emplace_back(cv::Point3d(p[0], p[1], p[2]));
  }

  std::vector<cv::viz::Widget> widgets;

  // add text widgets
  if (draw_point_id_) {
    int32_t id_count = 0;
    for (const auto& p : points_world_frame_) {
      widgets.emplace_back(cv::viz::WText3D(std::to_string(id_count++),
                                            cv::Point3d(p[0], p[1], p[2])));
    }
  }

  if (colors_.empty()) {
    widgets.emplace_back(cv::viz::WCloud(point_cloud));
    return widgets;
  } else {
    std::vector<cv::Vec3b> color_cloud;
    color_cloud.reserve(colors_.size());
    if (colors_.size() == points_world_frame_.size()) {
      for (const auto& c : colors_) {
        color_cloud.emplace_back(cv::Vec3b(c[0], c[1], c[2]));
      }
    } else {
      for (size_t i = 0; i < point_cloud.size(); i++) {
        color_cloud.emplace_back(
            cv::Vec3b(colors_[0][0], colors_[0][1], colors_[0][2]));
      }
    }
    return {cv::viz::WCloud(point_cloud, color_cloud)};
  }
}

cv::Affine3d vslam::viewer::PointCloudPrimitive::GetPose() const {
  return cv::Affine3d::Identity();
}

vslam::viewer::PointCloudPrimitive* vslam::viewer::PointCloudPrimitive::Clone()
    const {
  return new PointCloudPrimitive(*this);
}

Covariance2DPrimitive* Covariance2DPrimitive::Clone() const {
  return new Covariance2DPrimitive(*this);
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

std::vector<cv::viz::Widget> vslam::viewer::CameraPosePrimitive::GetWidget()
    const {
  cv::Matx33d cv_intrinsic_matrix;
  cv::eigen2cv(intrinsic_, cv_intrinsic_matrix);
  cv::viz::WCameraPosition w_camera_position(
      cv_intrinsic_matrix, 1.0, cv::Scalar(color_[0], color_[1], color_[2]));
  return {w_camera_position};
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

vslam::viewer::CoordinateSystemPrimitive::CoordinateSystemPrimitive(
    const std::string& tag_name,
    const vslam::Pose_t& pose_world_to_current)
    : PrimitiveBase(),
      tag_name_(tag_name),
      position_world_frame_(pose_world_to_current.translation()),
      orientation_world_frame_(pose_world_to_current.rotationMatrix()) {}

std::string vslam::viewer::CoordinateSystemPrimitive::GetTag() const {
  return tag_name_;
}
std::vector<cv::viz::Widget>
vslam::viewer::CoordinateSystemPrimitive::GetWidget() const {
  return {cv::viz::WCoordinateSystem()};
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

QuadricPrimitive::QuadricPrimitive(const std::string& tag_name,
                                   const vslam::Vec3_t& quadric_scale,
                                   const vslam::Vec3_t& position_world_frame,
                                   const vslam::Quat_t& orientation_world_frame,
                                   const vslam::Vec3_t& color)
    : PrimitiveBase(),
      tag_name_(tag_name),
      quadric_scale_(quadric_scale),
      position_world_frame_(position_world_frame),
      orientation_world_frame_(orientation_world_frame),
      color_(color) {}

std::string QuadricPrimitive::GetTag() const { return tag_name_; }
std::vector<cv::viz::Widget> QuadricPrimitive::GetWidget() const {
  return {
      WQuadric(quadric_scale_, cv::Scalar(color_[0], color_[1], color_[2]))};

  //  return cv::viz::WSphere({0,0,0}, 1);
}
cv::Affine3d QuadricPrimitive::GetPose() const {
  cv::Mat tmp_camera_attitude;
  cv::eigen2cv(orientation_world_frame_.toRotationMatrix(),
               tmp_camera_attitude);
  cv::Affine3d tmp_cam_pose(tmp_camera_attitude,
                            cv::Vec3f(position_world_frame_[0],
                                      position_world_frame_[1],
                                      position_world_frame_[2]));
  return tmp_cam_pose;
}
QuadricPrimitive* QuadricPrimitive::Clone() const {
  return new QuadricPrimitive(*this);
}

CovariancePrimitive::CovariancePrimitive(
    const std::string& tag_name,
    const vslam::Vec3_t& position_world_frame,
    const vslam::Quat_t& orientation_world_T_current,
    const vslam::Mat33_t& covariance_current_frame,
    const vslam::Vec3_t& color,
    double opacity,
    double chi_chi)
    : PrimitiveBase(),
      tag_name_(tag_name),
      position_world_frame_(position_world_frame),
      orientation_world_T_current_(orientation_world_T_current),
      covariance_current_frame_(covariance_current_frame),
      color_(color),
      opacity_(opacity),
      chi_chi_(chi_chi) {
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigensolver(
      covariance_current_frame_);
  //  Eigen::EigenSolver<Eigen::Matrix3d>
  //  eigensolver(covariance_current_frame_);
  if (eigensolver.info() != Eigen::Success) {
    ellipsoid_scale_ = {0.0, 0.0, 0.0};
    rotation_current_T_ellipsoid_ = vslam::Mat33_t::Identity();
    spdlog::warn("{}:{} Failed to compute eigen values.");
  }
  cout << "The eigenvalues of A are:\n" << eigensolver.eigenvalues() << endl;
  cout << "Here's a matrix whose columns are eigenvectors of A \n"
       << "corresponding to these eigenvalues:\n"
       << eigensolver.eigenvectors() << endl;

  //  ellipsoid_scale_ =
  //  vslam::Vec3_t(std::sqrt(eigensolver.eigenvalues()[0].real() * chi_chi_),
  //                                   std::sqrt(eigensolver.eigenvalues()[1].real()
  //                                   * chi_chi_),
  //                                   std::sqrt(eigensolver.eigenvalues()[2].real()
  //                                   * chi_chi_));
  ellipsoid_scale_ = vslam::Vec3_t(
      std::sqrt(std::abs(eigensolver.eigenvalues()[0]) * chi_chi_),
      std::sqrt(std::abs(eigensolver.eigenvalues()[1]) * chi_chi_),
      std::sqrt(std::abs(eigensolver.eigenvalues()[2]) * chi_chi_));

  rotation_current_T_ellipsoid_
      << eigensolver.eigenvectors().col(0).normalized(),
      eigensolver.eigenvectors().col(1).normalized(),
      eigensolver.eigenvectors().col(2).normalized();

  //  rotation_current_T_ellipsoid_ << eigensolver.eigenvectors();
  // normalize rotation matrix
  //  rotation_current_T_ellipsoid_ =
  //  vslam::Quat_t(rotation_current_T_ellipsoid_)
  //                                      .normalized()
  //                                      .toRotationMatrix();

  cout << "Ellipsoid scale : \n" << ellipsoid_scale_ << std::endl;
  cout << "Rotation mat : \n" << rotation_current_T_ellipsoid_ << std::endl;
}

std::string CovariancePrimitive::GetTag() const { return tag_name_; }
std::vector<cv::viz::Widget> CovariancePrimitive::GetWidget() const {
  auto widget =
      WQuadric(ellipsoid_scale_, cv::Scalar(color_[0], color_[1], color_[2]));
  widget.setRenderingProperty(cv::viz::OPACITY, opacity_);
  //  widget.setRenderingProperty(cv::viz::REPRESENTATION,
  //  cv::viz::REPRESENTATION_POINTS);
  return {widget};
}
cv::Affine3d CovariancePrimitive::GetPose() const {
  cv::Mat tmp_camera_attitude;
  vslam::Mat33_t rotation_world_T_ellipsoid =
      orientation_world_T_current_.toRotationMatrix() *
      rotation_current_T_ellipsoid_;
  //  vslam::Mat33_t rotation_world_T_ellipsoid = rotation_current_T_ellipsoid_;
  cv::eigen2cv(rotation_world_T_ellipsoid, tmp_camera_attitude);
  cv::Affine3d tmp_cam_pose(tmp_camera_attitude,
                            cv::Vec3f(position_world_frame_[0],
                                      position_world_frame_[1],
                                      position_world_frame_[2]));
  return tmp_cam_pose;
}
CovariancePrimitive* CovariancePrimitive::Clone() const {
  return new CovariancePrimitive(*this);
}

Covariance2DPrimitive::Covariance2DPrimitive(
    const std::string& tag_name,
    const vslam::Vec2_t& position_world_frame,
    double heading_world_T_current,
    const vslam::Mat22_t& covariance_current_frame,
    const vslam::Vec3_t& color,
    double opacity,
    double chi_chi)
    : PrimitiveBase(),
      tag_name_(tag_name),
      position_world_frame_(position_world_frame[0],
                            position_world_frame[1],
                            0),
      orientation_world_T_current_(
          Eigen::AngleAxisd(heading_world_T_current, vslam::Vec3_t(0.0, 0, 1.0))
              .toRotationMatrix()),
      color_(color),
      opacity_(opacity),
      chi_chi_(chi_chi) {
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> eigensolver(
      covariance_current_frame);
  //  Eigen::EigenSolver<Eigen::Matrix3d>
  //  eigensolver(covariance_current_frame_);
  if (eigensolver.info() != Eigen::Success) {
    ellipsoid_scale_ = {0.0, 0.0, 0.0};
    rotation_current_T_ellipsoid_ = vslam::Mat33_t::Identity();
    spdlog::warn("{}:{} Failed to compute eigen values.");
  }
  cout << "The eigenvalues of A are:\n" << eigensolver.eigenvalues() << endl;
  cout << "Here's a matrix whose columns are eigenvectors of A \n"
       << "corresponding to these eigenvalues:\n"
       << eigensolver.eigenvectors() << endl;

  //  ellipsoid_scale_ =
  //  vslam::Vec3_t(std::sqrt(eigensolver.eigenvalues()[0].real() * chi_chi_),
  //                                   std::sqrt(eigensolver.eigenvalues()[1].real()
  //                                   * chi_chi_),
  //                                   std::sqrt(eigensolver.eigenvalues()[2].real()
  //                                   * chi_chi_));
  ellipsoid_scale_ = vslam::Vec3_t(
      std::sqrt(std::abs(eigensolver.eigenvalues()[0]) * chi_chi_),
      std::sqrt(std::abs(eigensolver.eigenvalues()[1]) * chi_chi_),
      0);

  vslam::Mat22_t rotation_22;
  rotation_22 << eigensolver.eigenvectors().col(0).normalized(),
      eigensolver.eigenvectors().col(1).normalized();
  rotation_current_T_ellipsoid_ << rotation_22(0, 0), rotation_22(0, 1), 0,
      rotation_22(1, 0), rotation_22(1, 1), 0, 0, 0, 1;

  //  // normalize rotation matrix
  //  rotation_current_T_ellipsoid_ =
  //  vslam::Quat_t(rotation_current_T_ellipsoid_)
  //                                      .normalized()
  //                                      .toRotationMatrix();

  cout << "Ellipsoid scale : \n" << ellipsoid_scale_ << std::endl;
  cout << "Rotation mat : \n" << rotation_current_T_ellipsoid_ << std::endl;
}

std::string Covariance2DPrimitive::GetTag() const { return tag_name_; }

std::vector<cv::viz::Widget> Covariance2DPrimitive::GetWidget() const {
  auto widget =
      WQuadric(ellipsoid_scale_, cv::Scalar(color_[0], color_[1], color_[2]));
  widget.setRenderingProperty(cv::viz::OPACITY, opacity_);
  return {widget};
}

cv::Affine3d Covariance2DPrimitive::GetPose() const {
  cv::Mat tmp_camera_attitude;
  vslam::Mat33_t rotation_world_T_ellipsoid =
      rotation_current_T_ellipsoid_ *
      orientation_world_T_current_.toRotationMatrix();
  //  vslam::Mat33_t rotation_world_T_ellipsoid =
  //  orientation_world_T_current_.toRotationMatrix();
  cv::eigen2cv(rotation_world_T_ellipsoid, tmp_camera_attitude);
  cv::Affine3d tmp_cam_pose(tmp_camera_attitude,
                            cv::Vec3f(position_world_frame_[0],
                                      position_world_frame_[1],
                                      position_world_frame_[2]));
  return tmp_cam_pose;
}

TrajectoryPrimitive::TrajectoryPrimitive(
    const std::string& tag_name,
    const vslam::EigenAllocatedVector<vslam::Pose_t>& trajectory_world_frame,
    const vslam::Vec3_t& color,
    const DrawPrimitive& draw_primitive)
    : tag_name_(tag_name), color_(color), draw_primitive_(draw_primitive) {
  trajectory_affine.reserve(trajectory_world_frame.size());
  for (const auto& pose : trajectory_world_frame) {
    //    auto affine = cv::Mat_<double>(3,4);
    cv::Affine3d affine;
    affine.matrix << pose.matrix3x4()(0, 0), pose.matrix3x4()(0, 1),
        pose.matrix3x4()(0, 2), pose.matrix3x4()(0, 3), pose.matrix3x4()(1, 0),
        pose.matrix3x4()(1, 1), pose.matrix3x4()(1, 2), pose.matrix3x4()(1, 3),
        pose.matrix3x4()(2, 0), pose.matrix3x4()(2, 1), pose.matrix3x4()(2, 2),
        pose.matrix3x4()(2, 3);
    trajectory_affine.emplace_back(affine);
  }
}
std::string TrajectoryPrimitive::GetTag() const { return tag_name_; }
std::vector<cv::viz::Widget> TrajectoryPrimitive::GetWidget() const {
  cv::viz::Color traj_color(color_[0], color_[1], color_[2]);
  auto widget =
      cv::viz::WTrajectory(trajectory_affine, draw_primitive_, 1, traj_color);
  return {widget};
}
cv::Affine3d TrajectoryPrimitive::GetPose() const {
  return cv::Affine3d::Identity();
}
TrajectoryPrimitive* TrajectoryPrimitive::Clone() const {
  return new TrajectoryPrimitive(*this);
}
