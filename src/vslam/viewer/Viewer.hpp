//
// Created by ery on 2020/05/18.
//

#pragma once

#include <pangolin/pangolin.h>

namespace vslam::viewer {

class Viewer {
 public:
  explicit Viewer(const std::string& window_name = "Viewer",
                  int32_t window_width = 640,
                  int32_t window_height = 480);

  void LaunchViewer(bool is_block = false);

 private:
  void InitializeViewer(const std::string& window_name,
                        int32_t window_width,
                        int32_t window_height);
  void ViewerLoop();

  std::string window_name_;
  int32_t window_size_width_;
  int32_t window_size_height_;
};

}  // namespace vslam::viewer
