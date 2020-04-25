//
// Created by ery on 2020/04/25.
//

#pragma once

#include <mutex>

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

class ThreadsafeFrameContainer : ThreadsafeContainerBase {};

};  // namespace vslam::data
