//
// Created by ery on 2020/05/18.
//

#include <spdlog/spdlog.h>
#include <unistd.h>

#include <opencv2/calib3d.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/viz.hpp>

#include "ViewerViz.hpp"

int main() {

  auto viewer = vslam::viewer::ViewerViz();

  viewer.LaunchViewer(false);

  viewer.PushPrimitive(vslam::viewer::CoordinateSystemPrimitive(
      "world_origin", vslam::Vec3_t(0, 0, 0), vslam::Quat_t::Identity()));

  viewer.PushPrimitive(
      vslam::viewer::QuadricPrimitive("quadric_1",
                                      {1, 1, 0.0},
                                      {0, 0, 0},
                                      vslam::Quat_t::Identity(),
                                      {200, 1, 200}));

  vslam::Mat33_t orientation =
      Eigen::AngleAxisd(30.0 * M_PI / 180.0, vslam::Vec3_t(0.0, 0, 1.0))
          .toRotationMatrix();
  viewer.PushPrimitive(vslam::viewer::CoordinateSystemPrimitive(
      "pose1", vslam::Vec3_t(1, 1, 1), vslam::Quat_t(orientation)));

  vslam::Mat33_t covariance;
  covariance << 2.5, 0.75, 0.175, 0.75, 0.70, 0.135, 0.175, 01.35, 0.43;
  //  covariance << 2.5, 0.75, 0,
  //      0.75, 0.70, 0,
  //      0, 0, 0;
  viewer.PushPrimitive(
      vslam::viewer::CovariancePrimitive("cov",
                                         vslam::Vec3_t(1, 1, 1),
                                         vslam::Quat_t(orientation),
                                         covariance,
                                         {200, 1, 200}));

  vslam::EigenAllocatedVector<vslam::Vec3_t> points;
  points.emplace_back(vslam::Vec3_t{0, 0, 1});
  points.emplace_back(vslam::Vec3_t{0, 1, 0});
  points.emplace_back(vslam::Vec3_t{1, 0, 0});
  points.emplace_back(vslam::Vec3_t{0, 0, 1} * 2);
  points.emplace_back(vslam::Vec3_t{0, 1, 0} * 2);
  points.emplace_back(vslam::Vec3_t{1, 0, 0} * 2);
  for (size_t i = 0; i < 10000; i++) {
    points.emplace_back(vslam::Vec3_t{i, i, i});
    vslam::viewer::PointCloudPrimitive pcp("pc", points);
    viewer.PushPrimitive(pcp);

    spdlog::info("Publish point.");
    usleep(0.1 * 1e6);
  }
}
