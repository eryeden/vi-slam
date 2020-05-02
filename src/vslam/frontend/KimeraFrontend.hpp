//
// Created by ery on 2020/04/30.
//

#pragma once

#include "FrontendBase.hpp"

namespace vslam::frontend {

class KimeraFrontendInput {
 public:
  double timestamp_;
  cv::Mat frame_;
  data::PinholeCameraModel camera_model_;

 private:
};

class KimeraFrontend : public FrontendBase {
 public:
  KimeraFrontend(const std::shared_ptr<data::ThreadsafeMapDatabase>&
                     threadsafe_map_database);

  FrontendStatus Feed(const KimeraFrontendInput& frontend_input) override;

 private:
  data::Frame ProcessFirstFrame(const KimeraFrontendInput& frontend_input);
  data::Frame ProcessFrame(const KimeraFrontendInput& frontend_input,
                           const data::FrameSharedPtr& last_frame,
                           const data::FrameSharedPtr& last_keyframe);
  /**
   * @note
   * 自分用メモだが、SharedPrtは上書きでも参照回数がデクリメントされるので、
   * メモリが開放される。　https://beatsync.net/bayside/memo/log20050905.html
   *
   */
  data::FrameSharedPtr last_frame_;
  data::FrameSharedPtr last_keyframe_;

  data::PinholeCameraModel pinhole_camera_model_;
  bool is_first_frame_;
};

}  // namespace vslam::frontend