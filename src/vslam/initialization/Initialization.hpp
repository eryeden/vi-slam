//
// Created by ery on 2020/06/01.
//

#pragma once

#include <spdlog/spdlog.h>

#include <Eigen/Eigen>
#include <opengv/relative_pose/CentralRelativeAdapter.hpp>
#include <opengv/relative_pose/methods.hpp>
#include <opengv/sac/Ransac.hpp>
#include <opengv/sac_problems/relative_pose/CentralRelativePoseSacProblem.hpp>
#include <opengv/triangulation/methods.hpp>

#include "Frame.hpp"
#include "type_defines.hpp"

namespace vslam::initialization {

/**
 * @brief 2枚のFrameから、相対的Pose、特徴点位置の情報を推定する
 * @param reference_frame[in] :
 * 基準Frame、出力するPoseは基準FrameをIとして出力する
 * @param current_frame[in] : 現在Frame
 * @param current_frame_pose[out] : 推定したCurrent frameのPose
 * @param estimated_landmark_position[out] : 基準Frameの座標系でのLandmark
 * position
 * @return : Current frame Poseの推定と、Landmark位置推定に成功したか？
 */
static inline bool TryInitialize(
    const data::FrameWeakPtr& reference_frame,
    const data::FrameWeakPtr& current_frame,
    Pose_t& current_frame_pose,
    vslam::EigenAllocatedUnorderedMap<database_index_t, Vec3_t>&
        estimated_landmark_position,
    double inlier_rate_threshold) {
  // OpenGVによる5pointRANSACを実行
  using ProblemMono =
      opengv::sac_problems::relative_pose::CentralRelativePoseSacProblem;
  using AdapterMono = opengv::relative_pose::CentralRelativeAdapter;
  opengv::sac::Ransac<ProblemMono> mono_ransac_;

  // set ransac parameters
  double threshold = 1.0 - std::cos(1.0 * 180.0 / M_PI);
  mono_ransac_.threshold_ = threshold;
  mono_ransac_.max_iterations_ = 100;
  mono_ransac_.probability_ = 0.99;

  // Setup bearing vectors for opengv
  auto reference_frame_ptr = reference_frame.lock();
  auto current_frame_ptr = current_frame.lock();
  opengv::bearingVectors_t bearings_reference, bearings_current;
  std::vector<database_index_t> intersection_indices;
  if (reference_frame_ptr && current_frame_ptr) {
    bearings_reference.reserve(
        reference_frame_ptr->observing_feature_point_in_device_.size());
    bearings_current.reserve(
        current_frame_ptr->observing_feature_point_in_device_.size());
    // get intersection landmark indices
    std::set_intersection(reference_frame_ptr->observing_feature_id_.begin(),
                          reference_frame_ptr->observing_feature_id_.end(),
                          current_frame_ptr->observing_feature_id_.begin(),
                          current_frame_ptr->observing_feature_id_.end(),
                          std::back_inserter(intersection_indices));
    for (const auto idx : intersection_indices) {
      opengv::bearingVector_t bearing_reference, bearing_current;
      bearing_reference =
          reference_frame_ptr->observing_feature_bearing_in_camera_frame_.at(
              idx);
      bearing_current =
          current_frame_ptr->observing_feature_bearing_in_camera_frame_.at(idx);
      bearings_reference.emplace_back(bearing_reference);
      bearings_current.emplace_back(bearing_current);
    }
  }

  // Setup ransac problem
  AdapterMono adapter_mono(bearings_reference, bearings_current);
  auto problem =
      std::make_shared<ProblemMono>(adapter_mono, ProblemMono::NISTER);

  // Update problem
  mono_ransac_.sac_model_ = problem;

  // Solve.
  if (!mono_ransac_.computeModel(0)) {
    spdlog::warn("{}:{} Failure: 5pt RANSAC could not find a solution.",
                 __FILE__,
                 __FUNCTION__);
    return false;
  }

  // Inlierの個数チェック
  double inlier_rate = static_cast<double>(mono_ransac_.inliers_.size()) /
                       static_cast<double>(intersection_indices.size());
  if (inlier_rate < inlier_rate_threshold) {
    spdlog::warn("{}:{} Insufficient inliers. {}%[{}/{}]",
                 __FILE__,
                 __FUNCTION__,
                 inlier_rate * 100,
                 mono_ransac_.inliers_.size(),
                 intersection_indices.size());
    return false;
  } else {
    spdlog::info("Inliers found: {}%[{}/{}]",
                 inlier_rate * 100,
                 mono_ransac_.inliers_.size(),
                 intersection_indices.size());
  }

  // Output InlierのLandmark IDを抽出する。
  std::set<database_index_t> verified_feature_indices;
  for (auto inlier_array_idx : mono_ransac_.inliers_) {
    database_index_t landmark_index = intersection_indices[inlier_array_idx];
    verified_feature_indices.insert(landmark_index);
  }
  opengv::transformation_t best_transformation =
      mono_ransac_.model_coefficients_;
  Rot_t rotation(best_transformation.block<3, 3>(0, 0));
  Vec3_t translation = best_transformation.block<3, 1>(0, 3);
  Pose_t outpose(rotation, translation);
  current_frame_pose = outpose;
  //  spdlog::info("Initialized : T:{}", translation.norm());

  // triangulate points
  opengv::relative_pose::CentralRelativeAdapter tri_adapter(
      bearings_reference, bearings_current, translation, rotation.matrix());
  vslam::EigenAllocatedUnorderedMap<database_index_t, Vec3_t> landmark_position;
  for (auto inlier_array_idx : mono_ransac_.inliers_) {
    opengv::point_t point =
        opengv::triangulation::triangulate(tri_adapter, inlier_array_idx);
    landmark_position[intersection_indices[inlier_array_idx]] = point;
    //    spdlog::info("TR:{}, {},{},{}",
    //    intersection_indices[inlier_array_idx], point[0], point[1], point[2]);
  }
  estimated_landmark_position = landmark_position;

  return true;
}

}  // namespace vslam::initialization