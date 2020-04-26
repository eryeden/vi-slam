//
// Created by ery on 2020/04/25.
//

#pragma once

#include <mutex>

#include "Frame.hpp"
#include "Landmark.hpp"
#include "type_defines.hpp"

namespace vslam::data {


class ThreadsafeContainerBase {
 public:
 protected:
  std::mutex mutex_;
};

class ThreadsafeMapPointContainer : public ThreadsafeContainerBase {
 public:
 private:
};

class ThreadsafeFrameContainer : ThreadsafeContainerBase {
 public:

  void RegisterFrame(const Frame& frame);

 private:
};

};  // namespace vslam::data
