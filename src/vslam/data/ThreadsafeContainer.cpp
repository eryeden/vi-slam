//
// Created by ery on 2020/04/25.
//
#include "ThreadsafeContainer.hpp"

using namespace vslam::data;

FrameUnorderedMap<uint64_t>&
vslam::data::ThreadsafeFrameContainer::GetFrameDatabase() const {}
FrameUnorderedMap<uint64_t>&
vslam::data::ThreadsafeFrameContainer::GetLatestFrame() const {}
void vslam::data::ThreadsafeFrameContainer::RegisterFrame(const Frame& frame) {}
