//
// Created by ery on 2020/05/18.
//

#pragma once

#include <atomic>
#include <opencv2/calib3d.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/viz.hpp>
#include <string>
#include <thread>

#include "ThreadsafeQueue.hpp"
#include "ViewerPrimitives.hpp"

namespace vslam::viewer {

class ViewerViz {
 private:
  class PrimitiveInfo {
   public:
    cv::viz::Widget widget_;
    cv::Affine3d pose_;
    bool is_draw_;
  };
  using PrimitiveDatabase = std::unordered_map<std::string, PrimitiveInfo>;

 public:
  explicit ViewerViz(const std::string& window_name = "Viewer",
                     int32_t window_width = 640,
                     int32_t window_height = 480);

  void LaunchViewer(bool is_block = false);

  void ShutDownViewer() { is_shutdown_ = true; };

  void PushPrimitive(const PrimitiveBase& input_primitive);

  std::atomic_bool is_shutdown_;

 private:
  void InitializeViewer(const std::string& window_name,
                        int32_t window_width,
                        int32_t window_height);
  void ViewerLoop();
  std::string window_name_;
  int32_t window_size_width_;
  int32_t window_size_height_;
  std::thread render_loop_;
  PrimitiveDatabase primitive_database_;
  ThreadsafeQueue<std::shared_ptr<PrimitiveBase>> threadsafe_primitive_queue_;
  cv::viz::Viz3d window_;
};

}  // namespace vslam::viewer
