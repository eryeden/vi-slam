//
// Created by ery on 2020/06/01.
//

#include "iSAM2Backend.hpp"

#include <opencv2/opencv.hpp>

#include "BackendUtilities.hpp"
#include "DataUtilities.hpp"
#include "GeneralProjectionFactor.hpp"
#include "Initialization.hpp"
#include "PoseInitialization.hpp"
#include "spdlog/spdlog.h"
#include "type_defines.hpp"

vslam::backend::iSAM2Backend::iSAM2Backend(
    const std::shared_ptr<data::ThreadsafeMapDatabase>& map_database,
    const Parameter& parameter)
    : BackendBase(map_database),
      parameter_(parameter),
      backend_state_(BackendState::BootStrap),
      latest_frame_id_(std::numeric_limits<database_index_t>::max()),
      latest_key_frame_id_(std::numeric_limits<database_index_t>::max()) {
  /**
   * @brief Initialize ISAM2
   */
  gtsam::ISAM2Params isam_2_params = parameter_.AsISAM2Params();

  //  gtsam::ISAM2GaussNewtonParams gauss_newton_params;
  //  gauss_newton_params.wildfireThreshold = 0.001;
  //  isam_2_params.optimizationParams = gauss_newton_params;
  //
  //  isam_2_params.setCacheLinearizedFactors(true);
  //  isam_2_params.relinearizeThreshold = 0.01;
  //  isam_2_params.relinearizeSkip = 1;
  //  isam_2_params.findUnusedFactorSlots = true;
  //  // isam_param->enablePartialRelinearizationCheck = true;
  //  //  isam_2_params.setEvaluateNonlinearError(false);  // only for debugging
  //  //  isam_2_params.enableDetailedResults = false;     // only for
  //  debugging. isam_2_params.factorization = gtsam::ISAM2Params::CHOLESKY;  //
  //  QR

  isam_2_ptr_ = std::make_shared<gtsam::ISAM2>(isam_2_params);
}

vslam::backend::BackendState vslam::backend::iSAM2Backend::SpinOnce() {
  // 更新があるときのみ以下を実行する。
  if (latest_frame_id_ == map_database_->latest_frame_id_) {
    return backend_state_;
  }

  // 0 Frame 目初期Frameに単位元の姿勢を設定。
  if (map_database_->latest_frame_id_ == 0) {
    latest_frame_id_ = 0;
    latest_key_frame_id_ = 0;
    // reference poseは単位元を設定
    auto ref_frame_ptr = map_database_->GetFrame(0).lock();
    if (ref_frame_ptr) {
      Pose_t identity_pose(Mat33_t::Identity(), {0, 0, 0});
      ref_frame_ptr->SetCameraPose(identity_pose);
    }
    return backend_state_;
  }

  if (backend_state_ == BackendState::BootStrap) {
    auto current_frame =
        map_database_->GetFrame(map_database_->latest_frame_id_).lock();
    if (current_frame) {
      if (!current_frame->is_keyframe_) {
        return backend_state_;
      }
    }

    //////////////////// Initialize Map //////////////////////////////
    database_index_t reference_frame_id = parameter_.reference_frame_id_;
    bool is_initialization_success = MapInitialization(
        map_database_,
        map_database_->GetFrame(reference_frame_id),
        map_database_->GetFrame(map_database_->latest_frame_id_),
        parameter_);
    if (is_initialization_success) {
      //
      vslam::backend::utility::InitializeISAM2(
          isam_2_ptr_,
          map_database_,
          map_database_->GetFrame(reference_frame_id),
          map_database_->GetFrame(map_database_->latest_frame_id_),
          parameter_.isam2_reprojection_noise_sigma_,
          parameter_.isam2_prior_pose_position_sigma_,
          parameter_.isam2_prior_pose_orientation_sigma_,
          parameter_.isam2_iteration_number_);

      spdlog::info("{} : Succeed in initializing Map. Reference:{}, Current:{}",
                   __FUNCTION__,
                   reference_frame_id,
                   map_database_->latest_frame_id_);
      backend_state_ = BackendState::Nominal;

    } else {
      return backend_state_;
    }

  } else if (backend_state_ == BackendState::Nominal) {
    //////////////////// Update Map //////////////////////////////

    spdlog::info("{} : KeyFrame Input detected. Frame ID {}",
                 __FUNCTION__,
                 map_database_->latest_key_frame_id_);
    vslam::backend::utility::RegisterLandmarkObservation(
        map_database_,
        map_database_->GetFrame(map_database_->latest_key_frame_id_));

    auto current_frame =
        map_database_->GetFrame(map_database_->latest_frame_id_).lock();
    auto prev_frame =
        map_database_->GetFrame(map_database_->latest_frame_id_ - 1).lock();

    if (current_frame && prev_frame) {
      //////////////////// Estimate the Pose of Input Frame ///////////////////
      spdlog::info("{} : ########## Frame Pose Estimation ##########",
                   __FUNCTION__);

      // Frame Poseの推定
      auto estimated_pose = initialization::InitializePose(
          map_database_->GetFrame(map_database_->latest_frame_id_),
          map_database_,
          prev_frame->GetCameraPose(),
          parameter_.pose_initialization_ransac_threshold_,
          parameter_.pose_initialization_ransac_max_iterations_,
          parameter_.pose_initialization_ransac_probability_);

      if (estimated_pose != std::nullopt) {
        spdlog::info("{} : Succeed in p3p pose initialization.", __FUNCTION__);
        current_frame->SetCameraPose(estimated_pose.value());
        current_frame->internal_materials_.camera_pose_initial_ =
            estimated_pose.value();
      } else {
        spdlog::warn("{} : Failed in p3p pose initialization.", __FUNCTION__);
        current_frame->SetCameraPose(prev_frame->GetCameraPose());
        current_frame->internal_materials_.camera_pose_initial_ =
            prev_frame->GetCameraPose();
      }

      //      if (estimated_pose != std::nullopt) {
      //        spdlog::info("{} : Succeed in p3p pose initialization.",
      //        __FUNCTION__);
      //        current_frame->SetCameraPose(estimated_pose.value());
      //        current_frame->internal_materials_.camera_pose_initial_ =
      //        estimated_pose.value();

      auto refined_pose = initialization::RefinePose(
          map_database_->GetFrame(map_database_->latest_frame_id_),
          map_database_,
          parameter_.pose_refinement_reprojection_noise_sigma_,
          parameter_.pose_refinement_landmark_position_sigma_,
          parameter_.pose_refinement_use_previous_pose_factor_,
          parameter_.pose_refinement_previous_position_sigma_,
          parameter_.pose_refinement_previous_orientation_sigma_);

      if (refined_pose != std::nullopt) {
        spdlog::info("{} : Succeed in Pose Optimization.", __FUNCTION__);
        current_frame->SetCameraPose(refined_pose.value());
        current_frame->internal_materials_.camera_pose_optimized_ =
            refined_pose.value();
      }

      //      } else {
      //        return backend_state_;
      //      }

      //////////////////// Process KeyFrame //////////////////////////////
      // KeyFrameの時は追加でLandmarkPositionのTriangulate、iSAM2のUpdateを実行
      if (current_frame->is_keyframe_) {
        auto previous_key_frame =
            map_database_->GetFrame(latest_key_frame_id_).lock();
        if (previous_key_frame) {
          spdlog::info("{} : ########## Key Frame Triangulation ##########",
                       __FUNCTION__);
          vslam::EigenAllocatedUnorderedMap<database_index_t,
                                            vslam::data::LandmarkWeakPtr>
              triangulated_landmarks;

          //          TriangulateKeyFrame(
          //              map_database_,
          //              current_frame,
          //              previous_key_frame,
          //              triangulated_landmarks,
          //              parameter_.triangulation_reprojection_error_threshold_,
          //              parameter_.triangulation_minimum_parallax_threshold_);

          vslam::backend::utility::TriangulateKeyFrame(
              map_database_,
              current_frame,
              triangulated_landmarks,
              parameter_.triangulation_reprojection_error_threshold_,
              parameter_.triangulation_minimum_parallax_threshold_);

          spdlog::info("{} : ########## Update iSAM2 Observation ##########",
                       __FUNCTION__);

          vslam::backend::utility::UpdateISAMObservation(
              isam_2_ptr_,
              map_database_,
              triangulated_landmarks,
              parameter_.isam2_reprojection_noise_sigma_,
              parameter_.isam2_iteration_number_,
              parameter_.optimization_reprojection_error_threshold_);
        }
      }
    }

  } else {
    spdlog::warn("{}:{} Unrecognized Backend status:{}",
                 __FILE__,
                 __FUNCTION__,
                 backend_state_);
  }

  // frame_idの更新
  latest_frame_id_ = map_database_->latest_frame_id_;
  latest_key_frame_id_ = map_database_->latest_key_frame_id_;

  return backend_state_;
}

bool vslam::backend::iSAM2Backend::MapInitialization(
    std::shared_ptr<data::ThreadsafeMapDatabase>& map_database,
    vslam::data::FrameWeakPtr&& reference_frame,
    vslam::data::FrameWeakPtr&& current_frame,
    const Parameter& parameter) {
  // Mapの初期化を実施
  Pose_t outpose;
  vslam::EigenAllocatedUnorderedMap<database_index_t, Vec3_t>
      output_landmark_position;
  bool status_initiation =
      initialization::TryInitialize(reference_frame,
                                    current_frame,
                                    outpose,
                                    output_landmark_position,
                                    0.95,  // 0.95
                                    5);
  if (!status_initiation) {
    return false;
  }

  /**
   * @brief Poseを初期化
   */
  // current frameは推定したPoseを設定
  auto current_frame_ptr = current_frame.lock();
  if (current_frame_ptr) {
    current_frame_ptr->SetCameraPose(outpose);
    current_frame_ptr->is_keyframe_ = true;
  }
  // reference poseは単位元を設定
  auto ref_frame_ptr = reference_frame.lock();
  if (ref_frame_ptr) {
    Pose_t identity_pose(Mat33_t::Identity(), {0, 0, 0});
    ref_frame_ptr->SetCameraPose(identity_pose);
    ref_frame_ptr->is_keyframe_ = true;
  }

  /**
   * @brief Refine
   */
  bool refine_status = initialization::RefineInitializedMap(
      ref_frame_ptr, current_frame_ptr, output_landmark_position);
  if (!refine_status) {
    return false;
  }

  /**
   * @brief Landmarkを初期化
   */
  for (const auto& [id, pos] : output_landmark_position) {
    // Chrarity check
    if (current_frame_ptr) {
      Vec3_t pos_current_camera_frame =
          current_frame_ptr->GetCameraPose().inverse() * pos;
      if (pos_current_camera_frame[2] < 0) {
        spdlog::warn("{}:{} Landmark [{}] located behind the camera.",
                     __FILE__,
                     __FUNCTION__,
                     id);
        continue;
      }
    }

    if (map_database->IsExistLandmark(id)) {
      auto lm_ptr = map_database->GetLandmark(id).lock();
      if (lm_ptr) {
        lm_ptr->is_initialized_ = true;
        lm_ptr->SetLandmarkPosition(pos);
      } else {
        spdlog::warn("{}:{} Landmark [{}] dose exists but expired.",
                     __FILE__,
                     __FUNCTION__,
                     id);
      }
    } else {
      database_index_t lm_id = id;
      auto lm_ptr = std::unique_ptr<data::Landmark>(new data::Landmark(
          lm_id,
          std::set<database_index_t>{current_frame_ptr->frame_id_,
                                     ref_frame_ptr->frame_id_},
          pos,
          false,
          true));
      map_database->AddLandmark(lm_ptr);

      spdlog::info("{} : Add Landmark to DB [{}]", __FUNCTION__, id);
    }
  }

  return true;
}
