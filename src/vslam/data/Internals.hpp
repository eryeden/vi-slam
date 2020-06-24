//
// Created by ery on 2020/06/11.
//

#pragma once

#include "Camera.hpp"
#include "Landmark.hpp"
#include "type_defines.hpp"

namespace vslam::data {

class InternalMaterials {
 public:
  InternalMaterials() = default;
  //  InternalMaterials(const InternalMaterials& internal_materials);

  /// Dataset index of frame
  database_index_t frame_index_;
  /// Internal frame id
  database_index_t frame_id_;

  /**
   * @brief Observations
   */
  double timestamp_;
  bool is_keyframe_;
  std::set<database_index_t> observing_feature_id_;
  FeaturePositionDatabase observing_feature_point_in_device_;
  FeatureBearingDatabase observing_feature_bearing_in_camera_frame_;
  FeatureAgeDatabase feature_point_age_;

  Pose_t camera_pose_;
  Pose_t body_pose_;
  std::unordered_map<database_index_t, Landmark> landmarks_;

  /// Internals
  FeaturePositionDatabase features_pre_frame_;
  FeaturePositionDatabase features_after_tracking_;
  FeaturePositionDatabase features_after_detection_;
  FeaturePositionDatabase features_after_verification_;
  Pose_t camera_pose_initial_;
  Pose_t camera_pose_optimized_;
  Pose_t camera_pose_isam2_;

  /**
   * @brief Landmarks
   */
  /// Pose初期化に利用するLandmark
  std::unordered_map<database_index_t, Landmark> pose_initialization_landmarks_;
  /// Pose最適化に利用するLandmark
  std::unordered_map<database_index_t, Landmark> pose_optimization_landmarks_;
  /// このFrameでTriangulateできたLandmark
  std::unordered_map<database_index_t, Landmark> triangulated_landmarks_;
  /// このFrameでTriangulateし、最適化を行ったLandmark
  std::unordered_map<database_index_t, Landmark> optimized_landmarks_;
  /// 前回KeyFrameと共通して観測しているLandmark
  std::unordered_map<database_index_t, Landmark> take_over_landmarks_;
};

}  // namespace vslam::data