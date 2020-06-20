//
// Created by ery on 2020/06/12.
//

#include "iSAM2Backend.hpp"

using namespace vslam::backend;

vslam::backend::iSAM2Backend::Parameter::Parameter() {
  /// Initialization
  reference_frame_id_ = 0;

  //  3point ransac
  pose_initialization_ransac_threshold_ =
      0.1 * M_PI / 180.0;                            // 0.1 * M_PI / 180.0;
  pose_initialization_ransac_max_iterations_ = 100;  // 100;
  pose_initialization_ransac_probability_ = 0.99;    // 0.99;

  // motion only ba
  pose_refinement_reprojection_noise_sigma_ = 1.0;
  pose_refinement_landmark_position_sigma_ = 0.1;
  pose_refinement_use_previous_pose_factor_ = true;
  pose_refinement_previous_position_sigma_ = 0.1;
  pose_refinement_previous_orientation_sigma_ = 0.1;

  /// Keyframe & ISAM2
  // KeyFrame selection
  keyframe_min_frames_after_kf_ = 5;
  keyframe_new_kf_keypoints_threshold_ = 0.7;

  // Triangulation
  triangulation_reprojection_error_threshold_ = 5.0;
  triangulation_minimum_parallax_threshold_ = 1.0 * M_PI / 180.0;

  // isam2 params
  isam2_wildfire_threshold_ = 0.001;
  isam2_cache_linearized_factors_ = true;
  isam2_relinearize_threshold_ = 0.01;
  isam2_relinearize_skip_ = 1;
  isam2_find_unused_factor_slots_ = true;
  isam2_enable_partial_relinearization_check_ = true;  // only for debugging
  isam2_set_evaluate_nonlinear_error_ = false;         // only for debugging
  isam2_enable_detailed_results_ = false;              // only for debugging.
  isam2_factorization_ = gtsam::ISAM2Params::Factorization::CHOLESKY;

  //  gtsam::ISAM2GaussNewtonParams gauss_newton_params;
  //  gauss_newton_params.wildfireThreshold = 0.001;
  //  isam2_params_.optimizationParams = gauss_newton_params;
  //  isam2_params_.setCacheLinearizedFactors(true);
  //  isam2_params_.relinearizeThreshold = 0.01;
  //  isam2_params_.relinearizeSkip = 1;
  //  isam2_params_.findUnusedFactorSlots = true;
  //  //  isam2_params_.enablePartialRelinearizationCheck = true;
  //  //  isam2_params_.setEvaluateNonlinearError(false);  // only for debugging
  //  //  isam2_params_.enableDetailedResults = false;     // only for
  //  debugging. isam2_params_.factorization = gtsam::ISAM2Params::CHOLESKY;

  isam2_reprojection_noise_sigma_ = 0.1;
  isam2_prior_pose_position_sigma_ = 0.1;
  isam2_prior_pose_orientation_sigma_ = 0.1;
  isam2_iteration_number_ = 10;
  // Outlier rejection
  optimization_reprojection_error_threshold_ = 3.0;
}
gtsam::ISAM2Params iSAM2Backend::Parameter::AsISAM2Params() {
  gtsam::ISAM2Params params;

  gtsam::ISAM2GaussNewtonParams gauss_newton_params;
  gauss_newton_params.wildfireThreshold = this->isam2_wildfire_threshold_;
  params.optimizationParams = gauss_newton_params;

  params.setCacheLinearizedFactors(this->isam2_cache_linearized_factors_);
  params.relinearizeThreshold = this->isam2_relinearize_threshold_;
  params.relinearizeSkip = this->isam2_relinearize_skip_;
  params.findUnusedFactorSlots = this->isam2_find_unused_factor_slots_;
  //  params.enablePartialRelinearizationCheck =
  //  this->isam2_enable_partial_relinearization_check_; // only for debugging
  //  params.setEvaluateNonlinearError(this->isam2_set_evaluate_nonlinear_error_);
  //  // only for debugging params.enableDetailedResults =
  //  this->isam2_enable_detailed_results_;     // only for debugging.
  params.factorization = this->isam2_factorization_;

  return params;
}
