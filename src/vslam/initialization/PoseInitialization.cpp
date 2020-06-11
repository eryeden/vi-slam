//
// Created by ery on 2020/06/03.
//

#include "PoseInitialization.hpp"

#include <gtsam/geometry/Point2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/DoglegOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/ProjectionFactor.h>
#include <spdlog/spdlog.h>

#include <Eigen/Eigen>
#include <opengv/absolute_pose/CentralAbsoluteAdapter.hpp>
#include <opengv/absolute_pose/methods.hpp>
#include <opengv/sac/Ransac.hpp>
#include <opengv/sac_problems/absolute_pose/AbsolutePoseSacProblem.hpp>

#include "GeneralProjectionFactor.hpp"

using namespace vslam;
using namespace vslam::data;
using namespace vslam::initialization;
using namespace opengv;
using namespace gtsam;

std::optional<Pose_t> vslam::initialization::InitializePose(
    vslam::data::FrameWeakPtr&& input_frame,
    std::shared_ptr<data::ThreadsafeMapDatabase>& map_database,
    const Pose_t& previous_frame_pose) {
  bearingVectors_t bearings;
  points_t points;

  auto frame_ptr = input_frame.lock();
  auto lm_database = map_database->GetAllLandmarks();

  if (!frame_ptr) {
    return std::nullopt;
  }

  for (const auto& [id, bearing] :
       frame_ptr->observing_feature_bearing_in_camera_frame_) {
    if (lm_database.count(id) != 0) {
      auto lm_ptr = lm_database.at(id).lock();
      if (lm_ptr) {
        if (lm_ptr->is_initialized_ && !(lm_ptr->is_outlier_)) {
          bearings.emplace_back(bearing);
          points.emplace_back(lm_ptr->GetLandmarkPosition());
        }
      }
    }
  }

  if (bearings.empty()) {
    return std::nullopt;
  }

  // create a central absolute adapter
  absolute_pose::CentralAbsoluteAdapter adapter(
      bearings,
      points,
      previous_frame_pose.translation(),
      previous_frame_pose.rotationMatrix());
  //  absolute_pose::CentralAbsoluteAdapter adapter(
  //      bearings,
  //      points);

  // Create an AbsolutePoseSac problem and Ransac
  // The method can be set to KNEIP, GAO or EPNP
  sac::Ransac<sac_problems::absolute_pose::AbsolutePoseSacProblem> ransac;
  std::shared_ptr<sac_problems::absolute_pose::AbsolutePoseSacProblem>
      absposeproblem_ptr(
          new sac_problems::absolute_pose::AbsolutePoseSacProblem(
              adapter,
              sac_problems::absolute_pose::AbsolutePoseSacProblem::KNEIP));
  ransac.sac_model_ = absposeproblem_ptr;
  ransac.threshold_ = 1.0 - cos(0.0001 * M_PI / 180.0);
  ransac.max_iterations_ = 100;
  ransac.probability_ = 0.99;

  if (!ransac.computeModel()) {
    return std::nullopt;
  }

  opengv::transformation_t best_transformation = ransac.model_coefficients_;
  Rot_t rotation(best_transformation.block<3, 3>(0, 0));
  Vec3_t translation = best_transformation.block<3, 1>(0, 3);
  Pose_t outpose(rotation, translation);

  return std::optional<Pose_t>(outpose);
}
std::optional<Pose_t> vslam::initialization::RefinePose(
    FrameWeakPtr&& input_frame,
    std::shared_ptr<data::ThreadsafeMapDatabase>& map_database,
    bool use_previous_pose_factor) {
  auto frame_ptr = input_frame.lock();
  if (!frame_ptr) {
    return std::nullopt;
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
   * @brief Prior factorとしてLandmarkの位置を設定
   * @details
   * frameで観測されており、最適化済みのLandmarkをPriorFactorとして設定する
   */
  noiseModel::Diagonal::shared_ptr point_noise = noiseModel::Diagonal::Sigmas(
      (Vector(3) << Vector3::Constant(0.1)).finished());
  auto camera_model_ptr = std::shared_ptr<vslam::data::CameraModelBase>(
      frame_ptr->camera_model_->Clone());
  for (const auto& [id, pos] : frame_ptr->observing_feature_point_in_device_) {
    auto lm_ptr = map_database->GetLandmark(id).lock();
    if (lm_ptr) {
      // if (lm_ptr->is_added_ && !lm_ptr->is_outlier_)
      if (lm_ptr->is_initialized_ && !lm_ptr->is_outlier_) {
        // Frame観測情報を追加
        graph.emplace_shared<
            vslam::factor::GeneralProjectionFactor<Pose3, Point3>>(
            pos,
            measurement_noise,
            Symbol('x', 0),
            Symbol('l', id),
            camera_model_ptr);
        // LandmarkのPriorFactorを追加
        graph.emplace_shared<PriorFactor<Point3>>(
            Symbol('l', id),
            gtsam::Point3(lm_ptr->GetLandmarkPosition()),
            point_noise);
        initial_estimate.insert<gtsam::Point3>(
            Symbol('l', id), gtsam::Point3(lm_ptr->GetLandmarkPosition()));
      }
    }
  }

  if (use_previous_pose_factor) {
    auto previous_frame_ptr =
        map_database->GetFrame(map_database->latest_frame_id_ - 1).lock();
    if (previous_frame_ptr) {
      noiseModel::Diagonal::shared_ptr poseNoise = noiseModel::Diagonal::Sigmas(
          (Vector(6) << Vector3::Constant(0.1), Vector3::Constant(0.1))
              .finished());
      graph.emplace_shared<PriorFactor<Pose3>>(
          Symbol('x', 1),
          gtsam::Pose3(previous_frame_ptr->GetCameraPose().matrix()),
          poseNoise);
      Pose3 btw_pose = Pose3::identity();
      graph.emplace_shared<BetweenFactor<Pose3>>(
          Symbol('x', 0), Symbol('x', 1), btw_pose, poseNoise);
      initial_estimate.insert(
          Symbol('x', 1),
          gtsam::Pose3(previous_frame_ptr->GetCameraPose().matrix()));
    }
  }

  initial_estimate.insert(Symbol('x', 0),
                          gtsam::Pose3(frame_ptr->GetCameraPose().matrix()));

  /**
   * @brief 最適化の実施
   */
  try {
    /* Optimize the graph and print results */
    Values result = DoglegOptimizer(graph, initial_estimate).optimize();
    /**
     * @brief 結果の出力
     */
    auto current_pose = result.at(Symbol('x', 0)).cast<Pose3>();
    std::optional<Pose_t> out(current_pose.matrix());
    return out;
  } catch (gtsam::IndeterminantLinearSystemException& e) {
    spdlog::error("{}:{} IndeterminantLinearSystemException: {}",
                  __FILE__,
                  __FUNCTION__,
                  e.what());
    return std::nullopt;
  } catch (std::exception& e) {
    spdlog::error("{}:{} Solver error: {}", __FILE__, __FUNCTION__, e.what());
    return std::nullopt;
  }
}
