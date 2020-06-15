//
// Created by ery on 2020/06/01.
//

#include "iSAM2Backend.hpp"

#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/PriorFactor.h>

#include <opencv2/opencv.hpp>

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
      InitializeISAM2(isam_2_ptr_,
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
    RegisterLandmarkObservation(
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

          TriangulateKeyFrame(
              map_database_,
              current_frame,
              triangulated_landmarks,
              parameter_.triangulation_reprojection_error_threshold_,
              parameter_.triangulation_minimum_parallax_threshold_);

          spdlog::info("{} : ########## Update iSAM2 Observation ##########",
                       __FUNCTION__);

          UpdateISAMObservation(
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

void vslam::backend::iSAM2Backend::RegisterLandmarkObservation(
    std::shared_ptr<data::ThreadsafeMapDatabase>& map_database,
    const vslam::data::FrameWeakPtr& input_frame) {
  auto input_frame_ptr = input_frame.lock();
  if (input_frame_ptr) {
    for (const auto id : input_frame_ptr->observing_feature_id_) {
      if (map_database->IsExistLandmark(id)) {
        /**
         * @brief Landmarkが存在していれば被観測情報を更新する
         */
        auto lm_ptr = map_database->GetLandmark(id).lock();
        if (lm_ptr) {
          lm_ptr->SetObservedFrameIndex(input_frame_ptr->frame_id_);
        } else {
          spdlog::warn("{}:{}  Landmark exists but already expired.\n",
                       __FILE__,
                       __FUNCTION__);
        }

      } else {
        /**
         * @brief Landmarkが存在していないときは追加
         */
        database_index_t lm_id = id;  // map_database->max_landmark_id_ + 1;
        auto lm_ptr = std::unique_ptr<data::Landmark>(new data::Landmark(
            lm_id,
            std::set<database_index_t>{input_frame_ptr->frame_id_},
            {0, 0, 0},
            false,
            false));
        map_database->AddLandmark(lm_ptr);
      }
    }

  } else {
    spdlog::warn("{}:{} Input frame is expired\n", __FILE__, __FUNCTION__);
  }
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
                                    0.95,
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

using namespace gtsam;
bool vslam::backend::iSAM2Backend::InitializeISAM2(
    std::shared_ptr<gtsam::ISAM2>& isam_2,
    std::shared_ptr<data::ThreadsafeMapDatabase>& map_database,
    vslam::data::FrameWeakPtr&& reference_frame,
    vslam::data::FrameWeakPtr&& current_frame,
    double isam2_reprojection_noise_sigma,
    double isam2_prior_pose_position_sigma,
    double isam2_prior_pose_orientation_sigma,
    int32_t isam2_iteration_number) {
  auto reference_frame_ptr = reference_frame.lock();
  auto current_frame_ptr = current_frame.lock();
  if (!(reference_frame_ptr && current_frame_ptr)) {
    return false;
  }

  /**
   * @brief 観測モデルの定義
   * @note
   * Define the camera observation noise model
   * Isotropic error model
   * は円形の分布となっている。普通の二次元分布のように楕円状の分布ではない。
   */
  noiseModel::Isotropic::shared_ptr measurement_noise =
      noiseModel::Isotropic::Sigma(
          2, isam2_reprojection_noise_sigma);  // one pixel in u and v

  /**
   * @brief Factor graphの生成
   */
  NonlinearFactorGraph graph;
  Values initial_estimate;

  /**
   * @brief 座標系基準を設定
   */
  // Add a prior on pose x1. This indirectly specifies where the origin is.
  noiseModel::Diagonal::shared_ptr poseNoise = noiseModel::Diagonal::Sigmas(
      (Vector(6) << Vector3::Constant(isam2_prior_pose_position_sigma),
       Vector3::Constant(isam2_prior_pose_orientation_sigma))
          .finished());  // 30cm std on x,y,z 0.1 rad on roll,pitch,yaw
  gtsam::Pose3 identity_pose(Rot3::identity(), {0, 0, 0});
  graph.emplace_shared<PriorFactor<Pose3>>(
      Symbol('x', reference_frame_ptr->frame_id_),
      identity_pose,
      poseNoise);  // add directly to graph

  graph.emplace_shared<PriorFactor<Pose3>>(
      Symbol('x', current_frame_ptr->frame_id_),
      gtsam::Pose3(current_frame_ptr->GetCameraPose().matrix()),
      poseNoise);  // add directly to graph

  /**
   * @brief　観測情報の追加
   */
  auto camera_model_ptr = std::shared_ptr<vslam::data::CameraModelBase>(
      reference_frame_ptr->camera_model_->Clone());
  for (const auto& [id, lm_weak] : map_database->GetAllLandmarks()) {
    auto lm_ptr = lm_weak.lock();
    if (lm_ptr) {
      if (lm_ptr->is_initialized_ && !(lm_ptr->is_outlier_)) {
        graph.emplace_shared<
            vslam::factor::GeneralProjectionFactor<Pose3, Point3>>(
            reference_frame_ptr->observing_feature_point_in_device_.at(id),
            measurement_noise,
            Symbol('x', reference_frame_ptr->frame_id_),
            Symbol('l', id),
            camera_model_ptr);
        graph.emplace_shared<
            vslam::factor::GeneralProjectionFactor<Pose3, Point3>>(
            current_frame_ptr->observing_feature_point_in_device_.at(id),
            measurement_noise,
            Symbol('x', current_frame_ptr->frame_id_),
            Symbol('l', id),
            camera_model_ptr);
        gtsam::Point3 lm_point(lm_ptr->GetLandmarkPosition());
        initial_estimate.insert<gtsam::Point3>(Symbol('l', id), lm_point);
      }
    }
  }

  /**
   * @brief Frameの初期Poseを設定
   */
  initial_estimate.insert(
      Symbol('x', reference_frame_ptr->frame_id_),
      gtsam::Pose3(reference_frame_ptr->GetCameraPose().matrix()));
  initial_estimate.insert(
      Symbol('x', current_frame_ptr->frame_id_),
      gtsam::Pose3(current_frame_ptr->GetCameraPose().matrix()));

  /**
   * @brief Setup ISAM2 and run optimization
   */
  isam_2->update(graph, initial_estimate);
  //  Values currentEstimate = isam_2->calculateEstimate();
  //  spdlog::info("Opt[{}]", 0);
  //  isam_2->print("ISAM2 State");
  //  currentEstimate.print("Current estimate: ");
  for (size_t i = 0; i < isam2_iteration_number; i++) {
    /**
     * @brief Additional Optimization
     */
    isam_2->update();
    //    spdlog::info("Opt[{}]", i);
    //    currentEstimate = isam_2->calculateEstimate();
    //    isam_2->print("ISAM2 State");
    //    currentEstimate.print("Current estimate: ");
  }

  /**
   * @brief Update map_database
   */
  Values current_estimate = isam_2->calculateEstimate();
  auto reference_pose =
      current_estimate.at(Symbol('x', reference_frame_ptr->frame_id_))
          .cast<Pose3>();
  auto current_pose =
      current_estimate.at(Symbol('x', current_frame_ptr->frame_id_))
          .cast<Pose3>();
  reference_frame_ptr->SetCameraPose(Pose_t(reference_pose.matrix()));
  current_frame_ptr->SetCameraPose(Pose_t(current_pose.matrix()));
  for (const auto& [id, lm_weak] : map_database->GetAllLandmarks()) {
    auto lm_ptr = lm_weak.lock();
    if (lm_ptr) {
      if (lm_ptr->is_initialized_ && !(lm_ptr->is_outlier_)) {
        auto lm_pos = current_estimate.at(Symbol('l', id)).cast<Point3>();
        lm_ptr->SetLandmarkPosition(lm_pos);
        lm_ptr->is_added_ = true;
      }
    }
  }

  return true;
}

bool vslam::backend::iSAM2Backend::TriangulateKeyFrame(
    std::shared_ptr<data::ThreadsafeMapDatabase>& map_database,
    const data::FrameWeakPtr& current_key_frame,
    const data::FrameWeakPtr& previous_key_frame,
    vslam::EigenAllocatedUnorderedMap<database_index_t,
                                      vslam::data::LandmarkWeakPtr>&
        triangulated_landmarks,
    double reprojection_error_threshold,
    double minimum_parallax_threshold) {
  /**
   * @brief
   * 2つのKeyFrame間で共通して観測されていて、初期化されていないLandmarkをTriangulateする
   */
  auto current_ptr = current_key_frame.lock();
  auto previous_ptr = previous_key_frame.lock();
  auto lm_database = map_database->GetAllLandmarks();

  if (!current_ptr || !previous_ptr) {
    return false;
  }

  // 共通観測Landmarkを抽出
  std::vector<database_index_t> intersection_lm_ids, uninitialized_lm_lds;
  std::set_intersection(current_ptr->observing_feature_id_.begin(),
                        current_ptr->observing_feature_id_.end(),
                        previous_ptr->observing_feature_id_.begin(),
                        previous_ptr->observing_feature_id_.end(),
                        std::back_inserter(intersection_lm_ids));
  // 未初期化のLandmarkを抽出
  for (const auto id : intersection_lm_ids) {
    auto lm_ptr = lm_database[id].lock();
    if (lm_ptr) {
      if (!lm_ptr->is_initialized_) {
        uninitialized_lm_lds.emplace_back(id);
      }
    }
  }
  if (uninitialized_lm_lds.empty()) {
    spdlog::warn("{} : No landmarks to triangulate.", __FUNCTION__);
    return false;
  }

  // Triangulate
  opengv::bearingVectors_t bearings_current, bearings_previous;
  for (const auto id : uninitialized_lm_lds) {
    bearings_current.emplace_back(
        current_ptr->observing_feature_bearing_in_camera_frame_.at(id));
    bearings_previous.emplace_back(
        previous_ptr->observing_feature_bearing_in_camera_frame_.at(id));
  }

  Pose_t pose_previous_to_current =
      previous_ptr->GetCameraPose().inverse() * current_ptr->GetCameraPose();

  opengv::relative_pose::CentralRelativeAdapter tri_adapter(
      bearings_previous,
      bearings_current,
      pose_previous_to_current.translation(),
      pose_previous_to_current.rotationMatrix());

  vslam::EigenAllocatedUnorderedMap<database_index_t, Vec3_t> landmark_position;
  std::vector<double> landmark_observation_angle;

  for (size_t i = 0; i < uninitialized_lm_lds.size(); i++) {
    opengv::point_t point = opengv::triangulation::triangulate2(tri_adapter, i);

    Vec3_t ref_to_lm = (point - Vec3_t::Zero()).normalized();
    Vec3_t current_to_lm =
        (point - pose_previous_to_current.translation()).normalized();
    double angle = std::acos(ref_to_lm.dot(current_to_lm));
    landmark_observation_angle.emplace_back(angle);

    // measure reprojeciton error
    try {
      auto projected = previous_ptr->camera_model_->Project(point);
      auto measured = previous_ptr->observing_feature_point_in_device_.at(
          uninitialized_lm_lds[i]);
      auto measured_current =
          current_ptr->observing_feature_point_in_device_.at(
              uninitialized_lm_lds[i]);

      auto error = (measured - projected).norm();
      if ((error < reprojection_error_threshold) &&
          (angle > minimum_parallax_threshold)) {
        landmark_position[uninitialized_lm_lds[i]] =
            previous_ptr->GetCameraPose() * point;
      } else {
        spdlog::info("{} : Outlier detected LM:{}, {} [px], {} [deg]",
                     __FUNCTION__,
                     uninitialized_lm_lds[i],
                     error,
                     angle * 180.0 / M_PI);
      }
    } catch (vslam::data::ProjectionErrorException& exception) {
      spdlog::warn("{} : Projection failed LM:{} \nMessage: \n{}",
                   __FUNCTION__,
                   uninitialized_lm_lds[i],
                   exception.what());
    }
  }

  // Output triangulated landmarks
  for (const auto& [id, pos] : landmark_position) {
    auto lm_ptr = map_database->GetLandmark(id).lock();
    if (lm_ptr) {
      lm_ptr->SetLandmarkPosition(pos);
      lm_ptr->is_initialized_ = true;
      triangulated_landmarks[id] = map_database->GetLandmark(id);
    }
  }

  return true;
}

bool vslam::backend::iSAM2Backend::TriangulateKeyFrame(
    std::shared_ptr<data::ThreadsafeMapDatabase>& map_database,
    const data::FrameWeakPtr& current_key_frame,
    vslam::EigenAllocatedUnorderedMap<database_index_t,
                                      vslam::data::LandmarkWeakPtr>&
        triangulated_landmarks,
    double reprojection_error_threshold,
    double minimum_parallax_threshold) {
  /**
   * @brief
   * 2つのKeyFrame間で共通して観測されていて、初期化されていないLandmarkをTriangulateする
   */
  auto current_ptr = current_key_frame.lock();
  auto lm_database = map_database->GetAllLandmarks();

  if (!current_ptr) {
    return false;
  }

  // TriangulateされうるLandmarkを抽出
  std::vector<database_index_t> initializable_lm_ids;
  opengv::bearingVectors_t bearings_current, bearings_pared;
  EigenAllocatedVector<Pose_t> pose_current_T_pared;
  for (const auto id : current_ptr->observing_feature_id_) {
    auto lm_ptr = map_database->GetLandmark(id).lock();
    if (lm_ptr) {
      // CurrentFrameで観測されており、未初期化、２Frame以上から観測されているLandmarkを選択する
      bool is_initialized = lm_ptr->is_initialized_;
      int32_t observed_number = lm_ptr->GetAllObservedFrameIndex().size();
      if (!is_initialized && (observed_number >= 2)) {
        // 最も最初にLandmarkを観測したFrameをTriangulate対象に選ぶ
        database_index_t pared_frame_id =
            *(lm_ptr->GetAllObservedFrameIndex().begin());

        // CurrentFrameから、横方向に最も離れているFrameをTriangulate対象に選定する
        database_index_t max_baseline_frame_id = pared_frame_id;
        double max_baseline_length = 0;
        for (const auto tr_frame_id : lm_ptr->GetAllObservedFrameIndex()) {
          auto tr_frame_ptr = map_database->GetFrame(tr_frame_id).lock();
          if (tr_frame_ptr) {
            auto pose_current_T_tr = current_ptr->GetCameraPose().inverse() *
                                     tr_frame_ptr->GetCameraPose();
            //            auto bl_vec =
            //            Vec2_t(pose_current_T_tr.translation()[0],
            //            pose_current_T_tr.translation()[1]);
            auto bl_vec = pose_current_T_tr.translation();
            auto bl_len = bl_vec.norm();
            if (max_baseline_length < bl_len) {
              max_baseline_length = bl_len;
              max_baseline_frame_id = tr_frame_id;
            }
          }
        }

        auto pared_frame_ptr = map_database->GetFrame(pared_frame_id).lock();
        if (pared_frame_ptr) {
          initializable_lm_ids.emplace_back(id);
          bearings_current.emplace_back(
              current_ptr->observing_feature_bearing_in_camera_frame_.at(id));
          bearings_pared.emplace_back(
              pared_frame_ptr->observing_feature_bearing_in_camera_frame_.at(
                  id));
          //          pose_pared_T_current.emplace_back(pared_frame_ptr->GetCameraPose().inverse()
          //          * current_ptr->GetCameraPose());
          pose_current_T_pared.emplace_back(
              current_ptr->GetCameraPose().inverse() *
              pared_frame_ptr->GetCameraPose());
        }
      }
    }
  }

  if (initializable_lm_ids.empty()) {
    spdlog::warn("{} : No landmarks to triangulate.", __FUNCTION__);
    return false;
  }

  vslam::EigenAllocatedUnorderedMap<database_index_t, Vec3_t> landmark_position;
  std::vector<double> landmark_observation_angle;

  for (size_t i = 0; i < initializable_lm_ids.size(); i++) {
    opengv::relative_pose::CentralRelativeAdapter tri_adapter(
        {bearings_current[i]},
        {bearings_pared[i]},
        pose_current_T_pared[i].translation(),
        pose_current_T_pared[i].rotationMatrix());
    opengv::point_t point = opengv::triangulation::triangulate2(tri_adapter, 0);
    Vec3_t current_to_lm = (point - Vec3_t::Zero()).normalized();
    Vec3_t pared_to_lm =
        (point - pose_current_T_pared[i].translation()).normalized();
    double angle = std::acos(pared_to_lm.dot(current_to_lm));
    landmark_observation_angle.emplace_back(angle);

    // measure reprojeciton error
    try {
      Vec3_t point_world_frame = current_ptr->GetCameraPose() * point;

      auto projected = current_ptr->camera_model_->Project(point);
      auto measured = current_ptr->observing_feature_point_in_device_.at(
          initializable_lm_ids[i]);

      auto error = (measured - projected).norm();
      if ((error < reprojection_error_threshold) &&
          (angle > minimum_parallax_threshold)) {
        landmark_position[initializable_lm_ids[i]] = point_world_frame;
      } else {
        spdlog::info("{} : Outlier detected LM:{}, {} [px], {} [deg]",
                     __FUNCTION__,
                     initializable_lm_ids[i],
                     error,
                     angle * 180.0 / M_PI);
      }
    } catch (vslam::data::ProjectionErrorException& exception) {
      spdlog::warn("{} : Projection failed LM:{} \nMessage: \n{}",
                   __FUNCTION__,
                   initializable_lm_ids[i],
                   exception.what());
    }
  }

  // Output triangulated landmarks
  for (const auto& [id, pos] : landmark_position) {
    auto lm_ptr = map_database->GetLandmark(id).lock();
    if (lm_ptr) {
      lm_ptr->SetLandmarkPosition(pos);
      lm_ptr->is_initialized_ = true;
      triangulated_landmarks[id] = map_database->GetLandmark(id);
    }
  }

  return true;
}

bool vslam::backend::iSAM2Backend::UpdateISAMObservation(
    std::shared_ptr<gtsam::ISAM2>& isam_2,
    std::shared_ptr<data::ThreadsafeMapDatabase>& map_database,
    vslam::EigenAllocatedUnorderedMap<database_index_t,
                                      vslam::data::LandmarkWeakPtr>&
        triangulated_landmarks,
    double isam2_reprojection_noise_sigma,
    int32_t isam2_iteration_number,
    double reprojection_error_threshold) {
  auto current_frame_ptr =
      map_database->GetFrame(map_database->latest_key_frame_id_).lock();
  if (!current_frame_ptr) {
    spdlog::info("{} : Current frame expired.", __FUNCTION__);
    return false;
  }

  /**
   * @brief 観測モデルの定義
   * @note
   * Define the camera observation noise model
   * Isotropic error model
   * は円形の分布となっている。普通の二次元分布のように楕円状の分布ではない。
   */
  //  noiseModel::Isotropic::shared_ptr measurement_noise =
  //      noiseModel::Isotropic::Sigma(2, 0.1);  // one pixel in u and v
  noiseModel::Isotropic::shared_ptr measurement_noise =
      noiseModel::Isotropic::Sigma(
          2, isam2_reprojection_noise_sigma);  // one pixel in u and v

  /**
   * @brief Factor graphの生成
   */
  NonlinearFactorGraph graph;
  Values initial_estimate;

  /**
   * @brief　観測情報の追加
   */
  auto camera_model_ptr = std::shared_ptr<vslam::data::CameraModelBase>(
      current_frame_ptr->camera_model_->Clone());

  /**
   * @brief 新規観測Frameと既存観測LandmarkのProjeciton Factorを追加する
   */

  for (const auto id : current_frame_ptr->observing_feature_id_) {
    auto lm_ptr = map_database->GetLandmark(id).lock();
    if (lm_ptr) {
      if (lm_ptr->is_initialized_ && !lm_ptr->is_outlier_) {
        //        spdlog::info("{} : Add New KeyFrame Factor x:{}, l:{}",
        //                     __FUNCTION__,
        //                     current_frame_ptr->frame_id_,
        //                     id);
        graph.emplace_shared<
            vslam::factor::GeneralProjectionFactor<Pose3, Point3>>(
            current_frame_ptr->observing_feature_point_in_device_.at(id),
            measurement_noise,
            Symbol('x', current_frame_ptr->frame_id_),
            Symbol('l', id),
            camera_model_ptr);
      }
    }
  }

  /**
   * @brief 新たにTriangulateされたLandmarkのProjection Factorを追加する
   */
  for (const auto& [id, lm_weak_ptr] : triangulated_landmarks) {
    auto lm_ptr = lm_weak_ptr.lock();
    if (lm_ptr) {
      // lmを観測しているFrame全てのProjection Factorを追加する
      for (const auto observed_frame_id : lm_ptr->GetAllObservedFrameIndex()) {
        auto observed_frame_ptr =
            map_database->GetFrame(observed_frame_id).lock();
        if (observed_frame_ptr) {
          //          spdlog::info("{} : Add Newly Triangulated Factor x:{},
          //          l:{}",
          //                       __FUNCTION__,
          //                       observed_frame_ptr->frame_id_,
          //                       id);
          graph.emplace_shared<
              vslam::factor::GeneralProjectionFactor<Pose3, Point3>>(
              observed_frame_ptr->observing_feature_point_in_device_.at(id),
              measurement_noise,
              Symbol('x', observed_frame_ptr->frame_id_),
              Symbol('l', id),
              camera_model_ptr);
        }
      }

      // 初期値の追加
      gtsam::Point3 lm_point(lm_ptr->GetLandmarkPosition());
      initial_estimate.insert<gtsam::Point3>(Symbol('l', id), lm_point);
    }
  }

  /**
   * @brief Frameの初期Poseを設定
   */
  initial_estimate.insert(
      Symbol('x', current_frame_ptr->frame_id_),
      gtsam::Pose3(current_frame_ptr->GetCameraPose().matrix()));

  try {
    /**
     * @brief Setup ISAM2 and run optimization
     */
    isam_2->update(graph, initial_estimate);

  } catch (gtsam::IndeterminantLinearSystemException& e) {
    spdlog::error("{}:{} Solver IndeterminantLinearSystemException error: {}",
                  __FILE__,
                  __FUNCTION__,
                  e.what());
  } catch (std::exception& e) {
    spdlog::error("{}:{} Solver error: {}", __FILE__, __FUNCTION__, e.what());
  }

  for (size_t i = 0; i < isam2_iteration_number; i++) {
    /**
     * @brief Additional Optimization
     */
    try {
      isam_2->update();
    } catch (gtsam::IndeterminantLinearSystemException& e) {
      spdlog::error(
          "{}:{} Re calling Solver IndeterminantLinearSystemException error: "
          "{}",
          __FILE__,
          __FUNCTION__,
          e.what());
    } catch (std::exception& e) {
      spdlog::error("{}:{} Re calling Solver error: {}",
                    __FILE__,
                    __FUNCTION__,
                    e.what());
    }
  }

  /**
   * @brief Update map_database
   */
  Values current_estimate = isam_2->calculateEstimate();
  for (const auto& [id, data] : current_estimate) {
    Symbol symbol(id);
    Key key = symbol.chr();
    if (key == 'x') {
      auto frame_ptr = map_database->GetFrame(symbol.index()).lock();
      if (frame_ptr) {
        auto pose = Pose_t(data.cast<Pose3>().matrix());
        frame_ptr->SetCameraPose(pose);
      }
    } else if (key == 'l') {
      auto lm_ptr = map_database->GetLandmark(symbol.index()).lock();
      if (lm_ptr) {
        lm_ptr->SetLandmarkPosition(data.cast<Point3>());
        lm_ptr->is_added_ = true;
      }
    }
  }

  for (const auto id : current_frame_ptr->observing_feature_id_) {
    auto lm_ptr = map_database->GetLandmark(id).lock();
    if (lm_ptr) {
      if (lm_ptr->is_initialized_ && !lm_ptr->is_outlier_) {
        try {
          auto measured =
              current_frame_ptr->observing_feature_point_in_device_.at(id);
          auto projected = camera_model_ptr->Project(
              current_frame_ptr->GetCameraPose().inverse() *
              lm_ptr->GetLandmarkPosition());
          auto error = (projected - measured).norm();
          if (error > reprojection_error_threshold) {
            lm_ptr->is_outlier_ = true;
          }
        } catch (data::ProjectionErrorException& exception) {
          lm_ptr->is_outlier_ = true;
          spdlog::warn("Projection error: {}", exception.what());
        }
      }
    }
  }

  return true;
}