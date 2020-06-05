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

#include "GeneralProjectionFactor.hpp"
#include "Initialization.hpp"
#include "PoseInitialization.hpp"
#include "spdlog/spdlog.h"
#include "type_defines.hpp"

vslam::backend::iSAM2Backend::iSAM2Backend(
    const std::shared_ptr<data::ThreadsafeMapDatabase>& map_database)
    : BackendBase(map_database),
      backend_state_(BackendState::BootStrap),
      latest_frame_id_(std::numeric_limits<database_index_t>::max()),
      latest_key_frame_id_(std::numeric_limits<database_index_t>::max()) {
  /**
   * @brief Initialize ISAM2
   */
  gtsam::ISAM2Params isam_2_params;
  isam_2_params.relinearizeThreshold = 0.01;
  isam_2_params.relinearizeSkip = 1;
  isam_2_ptr_ = std::make_shared<gtsam::ISAM2>(isam_2_params);
}

vslam::backend::BackendState vslam::backend::iSAM2Backend::SpinOnce() {
  // 更新があるときのみ以下を実行する。
  if (latest_frame_id_ == map_database_->latest_frame_id_) {
    return backend_state_;
  }

  // Landmarkの被観測情報を更新・存在しないLandmarkは更新する
  RegisterLandmarkObservation(
      map_database_, map_database_->GetFrame(map_database_->latest_frame_id_));

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
    bool is_initialization_success = MapInitialization(
        map_database_,
        map_database_->GetFrame(0),
        map_database_->GetFrame(map_database_->latest_frame_id_));
    if (is_initialization_success) {
      InitializeISAM2(isam_2_ptr_,
                      map_database_,
                      map_database_->GetFrame(0),
                      map_database_->GetFrame(map_database_->latest_frame_id_));
      spdlog::info("Succeed in initializing Map.");
      backend_state_ = BackendState::Nominal;
    } else {
      return backend_state_;
    }
  } else if (backend_state_ == BackendState::Nominal) {
    auto current_frame =
        map_database_->GetFrame(map_database_->latest_frame_id_).lock();
    auto prev_frame =
        map_database_->GetFrame(map_database_->latest_frame_id_ - 1).lock();
    if (current_frame && prev_frame) {
      // Frame Poseの推定
      auto estimated_pose = initialization::InitializePose(
          map_database_->GetFrame(map_database_->latest_frame_id_),
          map_database_,
          prev_frame->GetCameraPose());
      if (estimated_pose != std::nullopt) {
        current_frame->SetCameraPose(estimated_pose.value());
        auto refined_pose = initialization::RefinePose(
            map_database_->GetFrame(map_database_->latest_frame_id_),
            map_database_);
        if (refined_pose != std::nullopt) {
          current_frame->SetCameraPose(refined_pose.value());
        }
      } else {
        return backend_state_;
      }

      // KeyFrameの時は追加でLandmarkPositionのTriangulate、iSAM2のUpdateを実行
      if (current_frame->is_keyframe_) {
        // 前回のKeyFrameとTriangulate
        auto previous_key_frame =
            map_database_->GetFrame(latest_key_frame_id_).lock();
        if (previous_key_frame) {
          TriangulateKeyFrame(map_database_, current_frame, previous_key_frame);
          ////          // ISAM2のUpdateを実施
          UpdateISAMObservation(isam_2_ptr_, map_database_, current_frame);
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
        database_index_t lm_id = map_database->max_landmark_id_ + 1;
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
    vslam::data::FrameWeakPtr&& current_frame) {
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
                                    10);
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
  initialization::RefineInitializedMap(
      ref_frame_ptr, current_frame_ptr, output_landmark_position);

  /**
   * @brief Landmarkを初期化
   */
  for (const auto& [id, pos] : output_landmark_position) {
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
      spdlog::warn(
          "{}:{} Landmark [{}] dose not exist.", __FILE__, __FUNCTION__, id);
    }
  }

  return true;
}

using namespace gtsam;
bool vslam::backend::iSAM2Backend::InitializeISAM2(
    std::shared_ptr<gtsam::ISAM2>& isam_2,
    std::shared_ptr<data::ThreadsafeMapDatabase>& map_database,
    vslam::data::FrameWeakPtr&& reference_frame,
    vslam::data::FrameWeakPtr&& current_frame) {
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
      noiseModel::Isotropic::Sigma(2, 1.0);  // one pixel in u and v

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
      (Vector(6) << Vector3::Constant(0.1), Vector3::Constant(0.1))
          .finished());  // 30cm std on x,y,z 0.1 rad on roll,pitch,yaw
  gtsam::Pose3 identity_pose(Rot3::identity(), {0, 0, 0});
  graph.emplace_shared<PriorFactor<Pose3>>(
      Symbol('x', 0), identity_pose, poseNoise);  // add directly to graph

  graph.emplace_shared<PriorFactor<Pose3>>(
      Symbol('x', 1),
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
            Symbol('x', 0),
            Symbol('l', id),
            camera_model_ptr);
        graph.emplace_shared<
            vslam::factor::GeneralProjectionFactor<Pose3, Point3>>(
            current_frame_ptr->observing_feature_point_in_device_.at(id),
            measurement_noise,
            Symbol('x', 1),
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
      Symbol('x', 0),
      gtsam::Pose3(reference_frame_ptr->GetCameraPose().matrix()));
  initial_estimate.insert(
      Symbol('x', 1),
      gtsam::Pose3(current_frame_ptr->GetCameraPose().matrix()));

  /**
   * @brief Setup ISAM2 and run optimization
   */
  isam_2->update(graph, initial_estimate);
  //  Values currentEstimate = isam_2->calculateEstimate();
  //  spdlog::info("Opt[{}]", 0);
  //  isam_2->print("ISAM2 State");
  //  currentEstimate.print("Current estimate: ");
  for (size_t i = 0; i < 1; i++) {
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
  auto reference_pose = current_estimate.at(Symbol('x', 0)).cast<Pose3>();
  auto current_pose = current_estimate.at(Symbol('x', 1)).cast<Pose3>();
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
    shared_ptr<data::ThreadsafeMapDatabase>& map_database,
    vslam::data::FrameWeakPtr&& current_key_frame,
    vslam::data::FrameWeakPtr&& previous_key_frame) {
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
  //  Pose_t pose_previous_to_current =
  //      current_ptr->GetCameraPose().inverse() *
  //      previous_ptr->GetCameraPose();
  Pose_t pose_previous_to_current =
      previous_ptr->GetCameraPose().inverse() * current_ptr->GetCameraPose();
  //  Pose_t pose_previous_to_current =  previous_ptr->GetCameraPose().inverse()
  //  * current_ptr->GetCameraPose();

  //  opengv::rotation_t p2c_rot(pose_previous_to_current.rotationMatrix());
  //  opengv::translation_t p2c_trans(pose_previous_to_current.translation());
  opengv::relative_pose::CentralRelativeAdapter tri_adapter(
      bearings_previous,
      bearings_current,
      pose_previous_to_current.translation(),
      pose_previous_to_current.rotationMatrix());

  vslam::EigenAllocatedUnorderedMap<database_index_t, Vec3_t> landmark_position;
  std::vector<double> landmark_observation_angle;
  cv::Mat debug_frame_prev(cv::Size(previous_ptr->camera_model_->width_,
                                    previous_ptr->camera_model_->height_),
                           CV_8UC3);
  cv::Mat debug_frame_current(cv::Size(previous_ptr->camera_model_->width_,
                                       previous_ptr->camera_model_->height_),
                              CV_8UC3);
  debug_frame_prev = 0;
  debug_frame_current = 0;
  for (size_t i = 0; i < uninitialized_lm_lds.size(); i++) {
    opengv::point_t point = opengv::triangulation::triangulate2(tri_adapter, i);

    Vec3_t ref_to_lm = (point - Vec3_t::Zero()).normalized();
    Vec3_t current_to_lm =
        (point - pose_previous_to_current.translation()).normalized();
    double angle = std::acos(ref_to_lm.dot(current_to_lm));
    landmark_observation_angle.emplace_back(angle);

    // measure reprojeciton error
    auto projected = previous_ptr->camera_model_->Project(point);
    auto measured = previous_ptr->observing_feature_point_in_device_.at(
        uninitialized_lm_lds[i]);
    auto measured_current = current_ptr->observing_feature_point_in_device_.at(
        uninitialized_lm_lds[i]);
    cv::circle(debug_frame_prev,
               cv::Point(measured[0], measured[1]),
               2,
               cv::Scalar(255, 255, 0));
    cv::putText(debug_frame_prev,
                std::to_string(uninitialized_lm_lds[i]),
                cv::Point(measured[0], measured[1]),
                cv::FONT_HERSHEY_PLAIN,
                0.5,
                cv::Scalar(0, 255, 0));
    cv::circle(debug_frame_current,
               cv::Point(measured_current[0], measured_current[1]),
               2,
               cv::Scalar(255, 255, 0));
    cv::putText(debug_frame_current,
                std::to_string(uninitialized_lm_lds[i]),
                cv::Point(measured_current[0], measured_current[1]),
                cv::FONT_HERSHEY_PLAIN,
                0.5,
                cv::Scalar(0, 255, 0));
    auto error = (measured - projected).norm();
    if ((error < 50.0) && (angle > 0.01)) {
      landmark_position[uninitialized_lm_lds[i]] =
          previous_ptr->GetCameraPose() * point;
    } else {
      spdlog::warn("Too large error, rejected. {} [px]", error);
      cv::circle(debug_frame_prev,
                 cv::Point(measured[0], measured[1]),
                 2,
                 cv::Scalar(0, 0, 255));
      cv::circle(debug_frame_current,
                 cv::Point(measured_current[0], measured_current[1]),
                 2,
                 cv::Scalar(0, 0, 255));
    }
  }
  cv::imshow("debug_prev", debug_frame_prev);
  cv::imshow("debug_current", debug_frame_current);
  // output
  for (const auto& [id, pos] : landmark_position) {
    auto lm_ptr = lm_database.at(id).lock();
    if (lm_ptr) {
      lm_ptr->SetLandmarkPosition(pos);
      lm_ptr->is_initialized_ = true;
    }
  }

  return true;
}

bool vslam::backend::iSAM2Backend::UpdateISAMObservation(
    shared_ptr<gtsam::ISAM2>& isam_2,
    shared_ptr<data::ThreadsafeMapDatabase>& map_database,
    vslam::data::FrameWeakPtr&& current_frame) {
  auto current_frame_ptr = current_frame.lock();
  if (!current_frame_ptr) {
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
      noiseModel::Isotropic::Sigma(2, 1.8);  // one pixel in u and v

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
  for (const auto id : current_frame_ptr->observing_feature_id_) {
    auto lm_ptr = map_database->GetLandmark(id).lock();
    if (lm_ptr) {
      if (lm_ptr->is_initialized_ && !lm_ptr->is_outlier_) {
        spdlog::info("Add x:{},l:{}", current_frame_ptr->frame_id_, id);
        graph.emplace_shared<
            vslam::factor::GeneralProjectionFactor<Pose3, Point3>>(
            current_frame_ptr->observing_feature_point_in_device_.at(id),
            measurement_noise,
            Symbol('x', current_frame_ptr->frame_id_),
            Symbol('l', id),
            camera_model_ptr);
        if (!lm_ptr->is_added_) {
          gtsam::Point3 lm_point(lm_ptr->GetLandmarkPosition());
          initial_estimate.insert<gtsam::Point3>(Symbol('l', id), lm_point);
        }
      }
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

  for (size_t i = 0; i < 1; i++) {
    /**
     * @brief Additional Optimization
     */
    try {
      isam_2->update();
    } catch (gtsam::IndeterminantLinearSystemException& e) {
      spdlog::error("{}:{} Solver IndeterminantLinearSystemException error: {}",
                    __FILE__,
                    __FUNCTION__,
                    e.what());
    } catch (std::exception& e) {
      spdlog::error("{}:{} Solver error: {}", __FILE__, __FUNCTION__, e.what());
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

  /**
   * @brief Outlier elimination
   */
  for (const auto id : current_frame_ptr->observing_feature_id_) {
    auto lm_ptr = map_database->GetLandmark(id).lock();
    if (lm_ptr) {
      if (lm_ptr->is_added_ && !lm_ptr->is_outlier_) {
        auto measured =
            current_frame_ptr->observing_feature_point_in_device_.at(id);
        auto projected = camera_model_ptr->Project(
            current_frame_ptr->GetCameraPose().inverse() *
            lm_ptr->GetLandmarkPosition());
        auto error = (projected - measured).norm();
        if (error > 3) {
          lm_ptr->is_outlier_ = true;
        }
      }
    }
  }

  return true;
}
