//
// Created by ery on 2020/06/11.
//

#pragma once

#include <cereal/archives/binary.hpp>
#include <cereal/archives/json.hpp>
#include <cereal/types/deque.hpp>
#include <cereal/types/memory.hpp>
#include <cereal/types/string.hpp>
#include <cereal/types/vector.hpp>

#include "Internals.hpp"

namespace cereal {

template <class Archive>
void serialize(Archive& ar, const vslam::data::InternalMaterials& p) {
  ar(cereal::make_nvp("frame_pose_initial", p.camera_pose_initial_),
     cereal::make_nvp("frame_pose_optimized", p.camera_pose_optimized_),
     cereal::make_nvp("frame_pose_isam2", p.camera_pose_isam2_));
}

}  // namespace cereal