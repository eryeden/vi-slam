//
// Created by ery on 2020/05/20.
//

#include <spdlog/spdlog.h>
#include <unistd.h>

#include <random>

#include "ViewerViz.hpp"

int main() {
  // generate sparse point and compute covariance
  vslam::EigenAllocatedVector<vslam::Vec3_t> points_world, points_body;
  vslam::Vec3_t position = {1.0, 2.0, 1.0};
  vslam::Vec3_t sigma = {1.2, 2.0, 0.7};
  vslam::Mat33_t orientation =
      Eigen::AngleAxisd(0.0 * M_PI / 180.0, vslam::Vec3_t(0, 1, 0))
          .toRotationMatrix() *
      Eigen::AngleAxisd(30.0 * M_PI / 180.0, vslam::Vec3_t(0, 0, 1))
          .toRotationMatrix();
  vslam::Mat33_t orientation_body =
      Eigen::AngleAxisd(30.0 * M_PI / 180.0, vslam::Vec3_t(0, 1, 0))
          .toRotationMatrix() *
      Eigen::AngleAxisd(30.0 * M_PI / 180.0, vslam::Vec3_t(0, 0, 1))
          .toRotationMatrix();
  int32_t point_num = 10000;
  std::random_device rnd;
  std::mt19937 mt(rnd());
  std::normal_distribution<> norm_x(0.0, sigma[0]);
  std::normal_distribution<> norm_y(0.0, sigma[1]);
  std::normal_distribution<> norm_z(0.0, sigma[2]);
  for (size_t i = 0; i < point_num; i++) {
    vslam::Vec3_t p{norm_x(mt), norm_y(mt), norm_z(mt)};
    points_body.emplace_back(orientation * p);
    points_world.emplace_back(orientation_body * orientation * p + position);
  }

  // compute covariance
  vslam::Mat33_t covariance_mat;
  double means[3] = {0, 0, 0};
  for (int i = 0; i < points_body.size(); i++) {
    means[0] += points_body[i].x(), means[1] += points_body[i].y(),
        means[2] += points_body[i].z();
  }
  means[0] /= points_body.size(), means[1] /= points_body.size(),
      means[2] /= points_body.size();
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      covariance_mat(i, j) = 0.0;
      for (int k = 0; k < points_body.size(); k++) {
        covariance_mat(i, j) +=
            (means[i] - points_body[k][i]) * (means[j] - points_body[k][j]);
      }
      covariance_mat(i, j) /= points_body.size() - 1;
    }
  }

  auto viewer = vslam::viewer::ViewerViz();

  viewer.LaunchViewer(false);

  viewer.PushPrimitive(vslam::viewer::CoordinateSystemPrimitive(
      "world_origin", vslam::Vec3_t(0, 0, 0), vslam::Quat_t::Identity()));

  viewer.PushPrimitive(vslam::viewer::PointCloudPrimitive("pc", points_world));

  //  viewer.PushPrimitive(
  //      vslam::viewer::PointCloudPrimitive("pc_body", points_body));

  viewer.PushPrimitive(
      vslam::viewer::CovariancePrimitive("cov",
                                         position,
                                         vslam::Quat_t(orientation_body),
                                         covariance_mat,
                                         {200, 0, 200},
                                         0.5));

  for (size_t i = 0; i < 10000; i++) {
    spdlog::info("Publish point.");
    usleep(0.1 * 1e6);
  }
}
