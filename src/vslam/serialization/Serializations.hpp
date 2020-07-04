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
#include "KimeraFrontend.hpp"
#include "iSAM2Backend.hpp"
#include "FeatureDetectorANMS.hpp"
#include "FeatureTrackerLSSDLucasKanade.hpp"
#include "EurocKimeraDataProvider.hpp"
#include "KittiKimeraDataProvider.hpp"

#include "basalt/serialization/eigen_io.h"

namespace cereal {

template <class Archive>
void serialize(Archive& ar, vslam::dataprovider::EurocKimeraDataProvider::Parameter& p) {
  ar(cereal::make_nvp("euroc_dataset_root", p.euroc_dataset_root_),
     cereal::make_nvp("ds_calibration_file", p.ds_calibration_file_),
     cereal::make_nvp("mask_image", p.mask_image_));
}

template <class Archive>
void serialize(Archive& ar, vslam::dataprovider::KittiKimeraDataProvider::Parameter& p) {
  ar(cereal::make_nvp("kitti_dataset_root", p.kitti_dataset_root_),
     cereal::make_nvp("mask_image", p.mask_image_));
}

template <class Archive>
void serialize(Archive& ar, vslam::data::Landmark& p) {
  ar(cereal::make_nvp("landmark_id", p.landmark_id_),

     cereal::make_nvp("is_initialized", p.is_initialized_),
     cereal::make_nvp("is_added", p.is_added_),
     cereal::make_nvp("is_outlier", p.is_outlier_),
     cereal::make_nvp("is_nearby", p.is_nearby_),

     cereal::make_nvp("observed_frame_id", p.GetAllObservedFrameIndex()),
     cereal::make_nvp("position_in_world", p.GetLandmarkPosition()));
}

template <class Archive>
void serialize(Archive& ar, vslam::data::InternalMaterials& p) {
  ar(cereal::make_nvp("frame_index", p.frame_index_),
     cereal::make_nvp("frame_id", p.frame_id_),
     cereal::make_nvp("timestamp", p.timestamp_),

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
     cereal::make_nvp("body_pose", p.body_pose_),
     cereal::make_nvp("landmarks", p.landmarks_),

     /// Internals
     cereal::make_nvp("features_pre_frame", p.features_pre_frame_),
     cereal::make_nvp("features_pre_frame_number",
                      p.features_pre_frame_.size()),
     cereal::make_nvp("features_after_tracking", p.features_after_tracking_),
     cereal::make_nvp("features_after_tracking_number",
                      p.features_after_tracking_.size()),
     cereal::make_nvp("features_after_detection", p.features_after_detection_),
     cereal::make_nvp("features_after_detection_number",
                      p.features_after_detection_.size()),
     cereal::make_nvp("features_after_verification",
                      p.features_after_verification_),
     cereal::make_nvp("features_after_verification_number",
                      p.features_after_verification_.size()),

     /// iSAM2に利用しているLandmarkの数

     cereal::make_nvp("pose_initialization_landmark_number",
                      p.pose_initialization_landmarks_.size()),
     cereal::make_nvp("pose_optimization_landmark_number",
                      p.pose_optimization_landmarks_.size()),
     cereal::make_nvp("triangulated_landmark_number",
                      p.triangulated_landmarks_.size()),
     cereal::make_nvp("optimized_landmark_number",
                      p.optimized_landmarks_.size()),
     cereal::make_nvp("take_over_landmark_number",
                      p.take_over_landmarks_.size()),
     cereal::make_nvp("nearby_landmark_number", p.nearby_landmarks_.size()),
     cereal::make_nvp("nearby_landmarks", p.nearby_landmarks_),

     /// Frame pose
     cereal::make_nvp("frame_pose_initial", p.camera_pose_initial_),
     cereal::make_nvp("frame_pose_optimized", p.camera_pose_optimized_),
     cereal::make_nvp("frame_pose_isam2", p.camera_pose_isam2_));
}

template <class Archive>
void serialize(Archive& ar, vslam::feature::FeatureDetectorANMS::Parameter& p) {
  ar(cereal::make_nvp("max_feature_number", p.max_feature_number_),
     cereal::make_nvp("min_feature_distance", p.min_feature_distance_),
     cereal::make_nvp("detection_feature_number", p.detection_feature_number_),
     cereal::make_nvp("detection_min_feature_distance",
                      p.detection_min_feature_distance_),
     cereal::make_nvp("detection_quality_level", p.detection_quality_level_),
     cereal::make_nvp("detection_block_size", p.detection_block_size_),
     cereal::make_nvp("detection_use_harris_corner",
                      p.detection_use_harris_corner_),
     cereal::make_nvp("detection_k", p.detection_k_),
     cereal::make_nvp("detection_ignore_edge", p.detection_ignore_edge_));
}

template <class Archive>
void serialize(Archive& ar,
               vslam::feature::FeatureTrackerLSSDLucasKanade::Parameter& p) {
  ar(cereal::make_nvp("optical_flow_type", p.optical_flow_type_),
     cereal::make_nvp("optical_flow_detection_grid_size",
                      p.optical_flow_detection_grid_size_),
     cereal::make_nvp("optical_flow_max_recovered_dist",
                      p.optical_flow_max_recovered_dist_),
     cereal::make_nvp("optical_flow_pattern", p.optical_flow_pattern_),
     cereal::make_nvp("optical_flow_max_iterations",
                      p.optical_flow_max_iterations_),
     cereal::make_nvp("optical_flow_levels", p.optical_flow_levels_));
}

template <class Archive>
void serialize(
    Archive& ar,
    vslam::verification::FeatureVerification5PointRANSAC::Parameter& p) {
  ar(cereal::make_nvp("ransac_threshold_angle_rad",
                      p.ransac_threshold_angle_rad_),
     cereal::make_nvp("ransac_max_iterations", p.ransac_max_iterations_),
     cereal::make_nvp("ransac_probability", p.ransac_probability_));
}

template <class Archive>
void serialize(Archive& ar, vslam::frontend::KimeraFrontend::Parameter& p) {
  ar(cereal::make_nvp("keyframe_interval", p.keyframe_interval_),
     cereal::make_nvp("minimum_keyframe_interval",
                      p.minimum_keyframe_interval_),
     cereal::make_nvp("low_keyframe_feature_number",
                      p.low_keyframe_feature_number_),
     cereal::make_nvp("counting_feature_age", p.counting_feature_age_));
}

template <class Archive>
void serialize(Archive& ar, vslam::backend::iSAM2Backend::Parameter& p) {
  ar(cereal::make_nvp("reference_frame_id", p.reference_frame_id_),

     cereal::make_nvp("keyframe_new_kf_keypoints_threshold",
                      p.keyframe_new_kf_keypoints_threshold_),
     cereal::make_nvp("keyframe_min_frames_after_kf",
                      p.keyframe_min_frames_after_kf_),

     cereal::make_nvp("pose_initialization_ransac_threshold",
                      p.pose_initialization_ransac_threshold_),
     cereal::make_nvp("pose_initialization_ransac_max_iterations",
                      p.pose_initialization_ransac_max_iterations_),
     cereal::make_nvp("pose_initialization_ransac_probability",
                      p.pose_initialization_ransac_probability_),
     cereal::make_nvp("pose_refinement_reprojection_noise_sigma",
                      p.pose_refinement_reprojection_noise_sigma_),
     cereal::make_nvp("pose_refinement_landmark_position_sigma",
                      p.pose_refinement_landmark_position_sigma_),
     cereal::make_nvp("pose_refinement_use_previous_pose_factor",
                      p.pose_refinement_use_previous_pose_factor_),
     cereal::make_nvp("pose_refinement_previous_position_sigma",
                      p.pose_refinement_previous_position_sigma_),
     cereal::make_nvp("pose_refinement_previous_orientation_sigma",
                      p.pose_refinement_previous_orientation_sigma_),

     cereal::make_nvp("triangulation_reprojection_error_threshold",
                      p.triangulation_reprojection_error_threshold_),
     cereal::make_nvp("triangulation_minimum_parallax_threshold",
                      p.triangulation_minimum_parallax_threshold_),

     cereal::make_nvp("isam2_reprojection_noise_sigma",
                      p.isam2_reprojection_noise_sigma_),
     cereal::make_nvp("isam2_prior_pose_position_sigma",
                      p.isam2_prior_pose_position_sigma_),
     cereal::make_nvp("isam2_prior_pose_orientation_sigma",
                      p.isam2_prior_pose_orientation_sigma_),
     cereal::make_nvp("isam2_iteration_number", p.isam2_iteration_number_),

     cereal::make_nvp("optimization_reprojection_error_threshold_",
                      p.optimization_reprojection_error_threshold_),

     cereal::make_nvp("isam2_wildfire_threshold", p.isam2_wildfire_threshold_),
     cereal::make_nvp("isam2_cache_linearized_factors",
                      p.isam2_cache_linearized_factors_),
     cereal::make_nvp("isam2_relinearize_threshold",
                      p.isam2_relinearize_threshold_),
     cereal::make_nvp("isam2_relinearize_skip", p.isam2_relinearize_skip_),
     cereal::make_nvp("isam2_find_unused_factor_slots",
                      p.isam2_find_unused_factor_slots_),
     cereal::make_nvp("isam2_enable_partial_relinearization_check",
                      p.isam2_enable_partial_relinearization_check_),
     cereal::make_nvp("isam2_set_evaluate_nonlinear_error",
                      p.isam2_set_evaluate_nonlinear_error_),
     cereal::make_nvp("isam2_enable_detailed_results_",
                      p.isam2_enable_detailed_results_),
     cereal::make_nvp("isam2_factorization", p.isam2_factorization_));
}

}  // namespace cereal