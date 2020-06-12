//
// Created by ery on 2020/06/12.
//

#include "iSAM2Backend.hpp"

using namespace vslam::backend;

vslam::backend::iSAM2Backend::Parameter::Parameter() {
  /// Initialization
  reference_frame_id_ = 0;

  //  5point ransac
  pose_initialization_ransac_threshold_ = 0.0001 * M_PI / 180.0;
  pose_initialization_ransac_max_iterations_ = 100;
  pose_initialization_ransac_probability_ = 0.99;

  // motion only ba
  pose_refinement_reprojection_noise_sigma_ = 1.0;
  pose_refinement_landmark_position_sigma_ = 0.1;
  pose_refinement_use_previous_pose_factor_ = true;
  pose_refinement_previous_position_sigma_ = 0.1;
  pose_refinement_previous_orientation_sigma_ = 0.1;

  /// Keyframe & ISAM2
  // Triangulation
  triangulation_reprojection_error_threshold_ = 5.0;
  triangulation_minimum_parallax_threshold_ = 1.0 * M_PI / 180.0;

  // isam2 params
  gtsam::ISAM2GaussNewtonParams gauss_newton_params;
  gauss_newton_params.wildfireThreshold = 0.001;
  isam2_params_.optimizationParams = gauss_newton_params;
  isam2_params_.setCacheLinearizedFactors(true);
  isam2_params_.relinearizeThreshold = 0.01;
  isam2_params_.relinearizeSkip = 1;
  isam2_params_.findUnusedFactorSlots = true;
  //  isam2_params_.enablePartialRelinearizationCheck = true;
  //  isam2_params_.setEvaluateNonlinearError(false);  // only for debugging
  //  isam2_params_.enableDetailedResults = false;     // only for debugging.
  isam2_params_.factorization = gtsam::ISAM2Params::CHOLESKY;

  isam2_reprojection_noise_sigma_ = 0.1;
  isam2_prior_pose_position_sigma_ = 0.1;
  isam2_prior_pose_orientation_sigma_ = 0.1;
  isam2_iteration_number_ = 10;
  // Outlier rejection
  optimization_reprojection_error_threshold_ = 3.0;
}