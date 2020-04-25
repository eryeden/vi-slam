//
// Created by ery on 2020/04/25.
//

#pragma once

#include <mutex>

#include "Frame.hpp"
#include "Landmark.hpp"
#include "type_defines.hpp"

namespace vslam::data {

using FrameDatabase = FrameUnorderedMap<uint64_t>;
using LandmarkDatabase = LandmarkUnorderedMap<uint64_t>;

class ThreadsafeContainerBase {
 public:
 protected:
  std::mutex mutex_;
};

class ThreadsafeMapPointContainer : public ThreadsafeContainerBase {
 public:
 private:
  LandmarkUnorderedMap<uint64_t> map_points_database_;
};

class ThreadsafeFrameContainer : ThreadsafeContainerBase {
 public:
  FrameDatabase& GetFrameDatabase() const;
  FrameDatabase& GetLatestFrame() const;

  void RegisterFrame(const Frame& frame);

 private:
  uint64_t latest_frame_index_;
  FrameDatabase frame_database_;
};

};  // namespace vslam::data
