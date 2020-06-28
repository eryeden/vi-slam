//
// Created by ery on 2020/06/19.
//

#include "ContinuousDetectorBackend.hpp"

#include <spdlog/spdlog.h>

#include "BackendUtilities.hpp"
#include "Initialization.hpp"
#include "PoseInitialization.hpp"
#include "Verification.hpp"

using namespace vslam;
using namespace vslam::data;
using namespace vslam::backend;

vslam::backend::ContinuousDetectorBackend::ContinuousDetectorBackend(
    const std::shared_ptr<data::ThreadsafeMapDatabase>& map_database,
    const vslam::backend::iSAM2Backend::Parameter& parameter)
    : BackendBase(map_database),
      parameter_(parameter),
      backend_state_(BackendState::BootStrap),
      latest_frame_id_(std::numeric_limits<database_index_t>::max()),
      latest_key_frame_id_(std::numeric_limits<database_index_t>::max()) {
  /**
   * @brief Initialize ISAM2
   */
  gtsam::ISAM2Params isam_2_params = parameter_.AsISAM2Params();
  isam_2_ptr_ = std::make_shared<gtsam::ISAM2>(isam_2_params);
}

/**
 * @brief KeyFrameの判定はここで実施する
 * @return
 */

vslam::backend::BackendState
vslam::backend::ContinuousDetectorBackend::SpinOnce() {
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

      auto current_frame_ptr =
          map_database_->GetFrame(map_database_->latest_frame_id_).lock();
      if (current_frame_ptr) {
        latest_frame_id_ = current_frame_ptr->frame_id_;
        latest_key_frame_id_ = current_frame_ptr->frame_id_;
        map_database_->latest_key_frame_id_ = current_frame_ptr->frame_id_;
      }

      spdlog::info("{} : Succeed in initializing Map. Reference:{}, Current:{}",
                   __FUNCTION__,
                   reference_frame_id,
                   map_database_->latest_frame_id_);
      backend_state_ = BackendState::Nominal;

    } else {
      return backend_state_;
    }

  } else {
    //////////////////// Update Map //////////////////////////////
    //    vslam::backend::utility::RegisterLandmarkObservation(
    //        map_database_,
    //        map_database_->GetFrame(map_database_->latest_key_frame_id_));

    auto current_frame =
        map_database_->GetFrame(map_database_->latest_frame_id_).lock();
    auto prev_frame =
        map_database_->GetFrame(map_database_->latest_frame_id_ - 1).lock();

    if (current_frame && prev_frame) {
      spdlog::info("Input frame : id:{}", current_frame->frame_id_);
      //////////////////// Estimate the Pose of Input Frame ///////////////////
      spdlog::info("{} : ########## Frame Pose Estimation ##########",
                   __FUNCTION__);

      // For debug
      auto frame_internals = current_frame->internal_materials_;

      // Initialize Frame Pose
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

      // Estimate Frame Pose
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

      //////////////////// Process KeyFrame //////////////////////////////
      // KeyFrameの時は追加でLandmarkPositionのTriangulate、iSAM2のUpdateを実行
      if (current_frame->is_keyframe_) {
        vslam::backend::utility::RegisterLandmarkObservation(
            map_database_,
            map_database_->GetFrame(map_database_->latest_key_frame_id_));

        auto previous_key_frame =
            map_database_->GetFrame(latest_key_frame_id_).lock();
        if (previous_key_frame) {
          spdlog::info("{} : ########## Key Frame Triangulation ##########",
                       __FUNCTION__);

          // 前回のKeyFrameと共通して観測しているLandmarkを保存しておく
          // Current frameとPrev frameで共通して観測しており、かつ初期化済みのLM
          // これで、Nearbyが発生しうるTakeover featureの数を見たい
          // =>あんまりTakeover featureの数はあんまり関係ないように思える
          std::vector<database_index_t> intersection_lm_ids;
          std::set_intersection(
              current_frame->observing_feature_id_.begin(),
              current_frame->observing_feature_id_.end(),
              previous_key_frame->observing_feature_id_.begin(),
              previous_key_frame->observing_feature_id_.end(),
              std::back_inserter(intersection_lm_ids));
          for (const auto lm_id : intersection_lm_ids) {
            auto lm_ptr = map_database_->GetLandmark(lm_id).lock();
            if (lm_ptr) {
              if (lm_ptr->is_initialized_ && (!lm_ptr->is_outlier_)) {
                frame_internals.take_over_landmarks_.insert(
                    std::pair<database_index_t, data::Landmark>(lm_id,
                                                                *lm_ptr));
              }
            }
          }

          //          /**
          //           * @brief 前回KeyFrameと今回KeyFrameで5point
          //           ransacを行い、Feature verificationを実行
          //           */
          //          auto verification_params =
          //              vslam::verification::FeatureVerification5PointRANSAC::Parameter();
          //          verification_params.ransac_threshold_angle_rad_ = 1.0 *
          //          M_PI / 180.0;
          //           verification::FeatureVerification5PointRANSAC
          //           verification_5_point_ransac(verification_params);
          //          auto verified_frame =
          //          verification_5_point_ransac.RemoveOutlier(*previous_key_frame,
          //          *current_frame); auto verified_frame_unique =
          //          std::make_unique<data::Frame>(verified_frame);
          //          map_database_->AddFrame(verified_frame_unique);
          //          current_frame =
          //          map_database_->GetFrame(map_database_->latest_frame_id_).lock();

          /**
           * @brief LMのTriangulate
           */
          vslam::EigenAllocatedUnorderedMap<database_index_t,
                                            vslam::data::LandmarkWeakPtr>
              triangulated_landmarks;
          vslam::backend::utility::TriangulateKeyFrame(
              map_database_,
              current_frame,
              triangulated_landmarks,
              parameter_.triangulation_reprojection_error_threshold_,
              parameter_.triangulation_minimum_parallax_threshold_);
          spdlog::info("{} : Triangulate features {}",
                       __FUNCTION__,
                       triangulated_landmarks.size());

          // For debug
          // Triangulate した直後のLM位置を保存
          for (auto& [lm_id, lm_weak] : triangulated_landmarks) {
            auto lm_ptr = lm_weak.lock();
            if (lm_ptr) {
              frame_internals.triangulated_landmarks_.insert(
                  std::pair<database_index_t, Landmark>(lm_id, *lm_ptr));
            }
          }

          spdlog::info("{} : ########## Update iSAM2 Observation ##########",
                       __FUNCTION__);

          bool is_solver_success =
              vslam::backend::utility::UpdateISAMObservation(
                  isam_2_ptr_,
                  map_database_,
                  triangulated_landmarks,
                  parameter_.isam2_reprojection_noise_sigma_,
                  parameter_.isam2_iteration_number_,
                  parameter_.optimization_reprojection_error_threshold_);
          if (!is_solver_success) {
            backend_state_ = BackendState::SolverException;
          }
        }
      }
      // for debug
      current_frame->internal_materials_ = frame_internals;
    }
  }

  // frame_idの更新
  latest_frame_id_ = map_database_->latest_frame_id_;
  latest_key_frame_id_ = map_database_->latest_key_frame_id_;

  return backend_state_;
}
bool ContinuousDetectorBackend::MapInitialization(
    std::shared_ptr<data::ThreadsafeMapDatabase>& map_database,
    FrameWeakPtr&& reference_frame,
    FrameWeakPtr&& current_frame,
    const iSAM2Backend::Parameter& parameter) {
  // Mapの初期化を実施
  Pose_t outpose;
  vslam::EigenAllocatedUnorderedMap<database_index_t, Vec3_t>
      output_landmark_position;
  bool status_initiation =
      initialization::TryInitialize(reference_frame,
                                    current_frame,
                                    outpose,
                                    output_landmark_position,
                                    0.99,  // 0.95
                                    5      // 5
      );
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
