//
// Created by ery on 2020/05/05.
//

#include "Verification.hpp"

#include <spdlog/spdlog.h>

vslam::verification::FeatureVerification5PointRANSAC::
    FeatureVerification5PointRANSAC(double ransac_threshold_angle_rad,
                                    int32_t ransac_max_iterations,
                                    double ransac_probability) {
  // set ransac threshold
  double threshold = 1.0 - std::cos(ransac_threshold_angle_rad);
  mono_ransac_.threshold_ = threshold;
  mono_ransac_.max_iterations_ = ransac_max_iterations;
  mono_ransac_.probability_ = ransac_probability;
}
vslam::data::Frame
vslam::verification::FeatureVerification5PointRANSAC::RemoveOutlier(
    const vslam::data::Frame& frame_reference,
    const vslam::data::Frame& frame_current) {
  // Setup bearing vectors for opengv
  opengv::bearingVectors_t bearings_reference, bearings_current;
  bearings_reference.reserve(
      frame_reference.observing_feature_point_in_device_.size());
  bearings_current.reserve(
      frame_current.observing_feature_point_in_device_.size());
  // get intersection landmark indices
  std::vector<database_index_t> intersection_indices;
  std::set_intersection(frame_reference.observing_feature_id_.begin(),
                        frame_reference.observing_feature_id_.end(),
                        frame_current.observing_feature_id_.begin(),
                        frame_current.observing_feature_id_.end(),
                        std::back_inserter(intersection_indices));
  for (const auto idx : intersection_indices) {
    opengv::bearingVector_t bearing_a, bearing_b;
    Vec2_t p_a_in_device =
        frame_reference.observing_feature_point_in_device_.at(idx);
    Vec2_t p_b_in_device =
        frame_current.observing_feature_point_in_device_.at(idx);
    bearing_a = {
        (frame_reference.camera_parameter_.cx - p_a_in_device[0]) /
            frame_reference.camera_parameter_.fx,
        (frame_reference.camera_parameter_.cy - p_a_in_device[1]) /
            frame_reference.camera_parameter_.fy,
        1,
    };
    bearing_a.normalize();
    bearing_b = {(frame_current.camera_parameter_.cx - p_b_in_device[0]) /
                     frame_current.camera_parameter_.fx,
                 (frame_current.camera_parameter_.cy - p_b_in_device[1]) /
                     frame_current.camera_parameter_.fy,
                 1};
    bearing_b.normalize();
    bearings_reference.emplace_back(bearing_a);
    bearings_current.emplace_back(bearing_b);
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
    return frame_current;
  }

  // Output
  FeatureAgeDatabase verified_feature_age_database;
  FeaturePositionDatabase verified_feeature_position_database;
  std::set<database_index_t> verified_feature_indices;

  for (auto inlier_array_idx : mono_ransac_.inliers_) {
    database_index_t landmark_index = intersection_indices[inlier_array_idx];
    verified_feature_indices.insert(landmark_index);
    verified_feeature_position_database[landmark_index] =
        frame_current.observing_feature_point_in_device_.at(landmark_index);
    verified_feature_age_database[landmark_index] =
        frame_current.feature_point_age_.at(landmark_index);
  }

  data::Frame output_frame(frame_current.frame_id_,
                           frame_current.timestamp_,
                           frame_current.is_keyframe_,
                           frame_current.camera_parameter_,
                           verified_feature_indices,
                           verified_feeature_position_database,
                           verified_feature_age_database);

  return output_frame;
}
