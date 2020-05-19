//
// Created by ery on 2020/05/18.
//

//#include "Viewer.hpp"
#include <spdlog/spdlog.h>
#include <unistd.h>

#include <opencv2/calib3d.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/viz.hpp>

#include "ViewerViz.hpp"

int main() {
  //  auto viewer = vslam::viewer::Viewer();
  //
  //  viewer.LaunchViewer(true);

  //  cv::viz::Viz3d myWindow("Coordinate Frame");
  //  myWindow.showWidget("Coordinate Widget", cv::viz::WCoordinateSystem());
  //
  //  myWindow.spin();

  auto viewer = vslam::viewer::ViewerViz();

  viewer.LaunchViewer(false);
  viewer.PushPrimitive(vslam::viewer::CoordinateSystemPrimitive(
      "world_origin", vslam::Vec3_t(0, 0, 0), vslam::Quat_t::Identity()));

  vslam::EigenAllocatedVector<vslam::Vec3_t> points;
  for (size_t i = 0; i < 10000; i++) {
    points.emplace_back(vslam::Vec3_t{i, i, i});
    vslam::viewer::PointCloudPrimitive pcp("pc", points);
    viewer.PushPrimitive(pcp);

    spdlog::info("Publish point.");
    usleep(0.1 * 1e6);
  }
}
