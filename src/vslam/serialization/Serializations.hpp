//
// Created by ery on 2020/06/11.
//

#pragma once

#include <cereal/archives/binary.hpp>
#include <cereal/archives/json.hpp>
#include <cereal/types/atomic.hpp>
#include <cereal/types/deque.hpp>
#include <cereal/types/map.hpp>
#include <cereal/types/memory.hpp>
#include <cereal/types/set.hpp>
#include <cereal/types/string.hpp>
#include <cereal/types/unordered_map.hpp>
#include <cereal/types/unordered_set.hpp>
#include <cereal/types/vector.hpp>

#include "Internals.hpp"
#include "basalt/serialization/eigen_io.h"

namespace cereal {

template <class Archive>
void serialize(Archive& ar, const vslam::data::Landmark& p) {
  ar(cereal::make_nvp("landmark_id", p.landmark_id_),

     cereal::make_nvp("is_initialized", p.is_initialized_),
     cereal::make_nvp("is_added", p.is_added_),
     cereal::make_nvp("is_outlier", p.is_outlier_),

     cereal::make_nvp("observed_frame_id", p.GetAllObservedFrameIndex()),
     cereal::make_nvp("position_in_world", p.GetLandmarkPosition()));
}

template <class Archive>
void serialize(Archive& ar, const vslam::data::InternalMaterials& p) {
  ar(cereal::make_nvp("frame_index", p.frame_index_),
     cereal::make_nvp("frame_id", p.frame_id_),

     cereal::make_nvp("is_keyframe", p.is_keyframe_),
     cereal::make_nvp("observing_feature_id", p.observing_feature_id_),
     cereal::make_nvp("observing_feature_point_in_device",
                      p.observing_feature_point_in_device_),
     cereal::make_nvp("observing_feature_bearing_in_camera_frame",
                      p.observing_feature_bearing_in_camera_frame_),
     cereal::make_nvp("feature_point_age", p.feature_point_age_),
     cereal::make_nvp("observing_feature_number",
                      p.observing_feature_point_in_device_.size()),
     cereal::make_nvp("camera_pose", p.camera_pose_),
     cereal::make_nvp("landmarks", p.landmarks_),

     /// Internals
     cereal::make_nvp("features_pre_frame", p.features_pre_frame_),
     cereal::make_nvp("features_pre_frame_number",
                      p.features_pre_frame_.size()),
     cereal::make_nvp("features_after_tracking", p.features_after_tracking_),
     cereal::make_nvp("features_after_tracking_number",
                      p.features_after_tracking_.size()),
     cereal::make_nvp("features_after_verification",
                      p.features_after_verification_),
     cereal::make_nvp("features_after_verification_number",
                      p.features_after_verification_.size()),

     cereal::make_nvp("frame_pose_initial", p.camera_pose_initial_),
     cereal::make_nvp("frame_pose_optimized", p.camera_pose_optimized_),
     cereal::make_nvp("frame_pose_isam2", p.camera_pose_isam2_));
}

}  // namespace cereal