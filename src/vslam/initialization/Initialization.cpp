//
// Created by ery on 2020/06/01.
//
#include "Initialization.hpp"

#include <gtsam/geometry/Point2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/DoglegOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/ProjectionFactor.h>
#include <spdlog/spdlog.h>

#include <Eigen/Eigen>
#include <opengv/relative_pose/CentralRelativeAdapter.hpp>
#include <opengv/relative_pose/methods.hpp>
#include <opengv/sac/Ransac.hpp>
#include <opengv/sac_problems/relative_pose/CentralRelativePoseSacProblem.hpp>
#include <utility>
#include <vector>

#include "GeneralProjectionFactor.hpp"

namespace vslam::initialization {

bool TryInitialize(const data::FrameWeakPtr& reference_frame,
                   const data::FrameWeakPtr& current_frame,
                   Pose_t& current_frame_pose,
                   vslam::EigenAllocatedUnorderedMap<database_index_t, Vec3_t>&
                       estimated_landmark_position,
                   double inlier_rate_threshold,
                   double parallax_threshold) {
  // OpenGVによる5pointRANSACを実行
  using ProblemMono =
      opengv::sac_problems::relative_pose::CentralRelativePoseSacProblem;
  using AdapterMono = opengv::relative_pose::CentralRelativeAdapter;
  opengv::sac::Ransac<ProblemMono> mono_ransac_;

  // set ransac parameters
  double threshold = 1.0 - std::cos(0.0001 * 180.0 / M_PI);
  mono_ransac_.threshold_ = threshold;
  mono_ransac_.max_iterations_ = 100;
  mono_ransac_.probability_ = 0.99;

  // Setup bearing vectors for opengv
  auto reference_frame_ptr = reference_frame.lock();
  auto current_frame_ptr = current_frame.lock();
  opengv::bearingVectors_t bearings_reference, bearings_current;
  std::vector<database_index_t> intersection_indices;
  double mean_parallax = 0;
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

      // 平均視差の計算
      mean_parallax +=
          (reference_frame_ptr->observing_feature_point_in_device_.at(idx) -
           current_frame_ptr->observing_feature_point_in_device_.at(idx))
              .norm();
    }
  }
  mean_parallax /= static_cast<double>(intersection_indices.size());

  // 初めに視差チェック
  if (mean_parallax < parallax_threshold) {
    spdlog::warn("{}:{} Insufficient parallax {}[px]",
                 __FILE__,
                 __FUNCTION__,
                 mean_parallax);
    return false;
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
  //  translation.normalize();
  Pose_t outpose(rotation, translation);
  current_frame_pose = outpose;
  //  spdlog::info("Initialized : T:{}", translation.norm());

  // triangulate points
  opengv::relative_pose::CentralRelativeAdapter tri_adapter(
      bearings_reference, bearings_current, translation, rotation.matrix());
  vslam::EigenAllocatedUnorderedMap<database_index_t, Vec3_t> landmark_position;
  for (auto inlier_array_idx : mono_ransac_.inliers_) {
    //    opengv::point_t point =
    //        opengv::triangulation::triangulate(tri_adapter, inlier_array_idx);
    opengv::point_t point =
        opengv::triangulation::triangulate2(tri_adapter, inlier_array_idx);
    landmark_position[intersection_indices[inlier_array_idx]] = point;
    //    spdlog::info("TR:{}, {},{},{}",
    //    intersection_indices[inlier_array_idx], point[0], point[1], point[2]);
  }

  estimated_landmark_position = landmark_position;
  return true;
}

using namespace gtsam;
bool RefineInitializedMap(const data::FrameWeakPtr& reference_frame,
                          const data::FrameWeakPtr& current_frame,
                          EigenAllocatedUnorderedMap<database_index_t, Vec3_t>&
                              estimated_landmark_position) {
  // Setup bearing vectors for opengv
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
  // Create a factor graph
  NonlinearFactorGraph graph;

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
   * @brief　感測情報の追加
   */
  auto camera_model_ptr = std::shared_ptr<vslam::data::CameraModelBase>(
      reference_frame_ptr->camera_model_->Clone());
  for (const auto& [id, pos] : estimated_landmark_position) {
    graph.emplace_shared<vslam::factor::GeneralProjectionFactor<Pose3, Point3>>(
        reference_frame_ptr->observing_feature_point_in_device_.at(id),
        measurement_noise,
        Symbol('x', 0),
        Symbol('l', id),
        camera_model_ptr);
    graph.emplace_shared<vslam::factor::GeneralProjectionFactor<Pose3, Point3>>(
        current_frame_ptr->observing_feature_point_in_device_.at(id),
        measurement_noise,
        Symbol('x', 1),
        Symbol('l', id),
        camera_model_ptr);
  }

  /**
   * @brief 初期値の設定
   */
  Values initial_estimate;
  //  gtsam::Pose3 ref_pose(reference_frame_ptr->GetCameraPose().matrix());
  initial_estimate.insert(
      Symbol('x', 0),
      gtsam::Pose3(reference_frame_ptr->GetCameraPose().matrix()));
  initial_estimate.insert(
      Symbol('x', 1),
      gtsam::Pose3(current_frame_ptr->GetCameraPose().matrix()));
  for (const auto& [id, pos] : estimated_landmark_position) {
    gtsam::Point3 lm_point(pos);
    initial_estimate.insert<gtsam::Point3>(Symbol('l', id), lm_point);
  }

  /**
   * @brief 最適化の実施
   */
  /* Optimize the graph and print results */
  Values result = DoglegOptimizer(graph, initial_estimate).optimize();
  //  result.print("Final results:\n");
  //  cout << "initial error = " << graph.error(initial_estimate) << endl;
  //  cout << "final error = " << graph.error(result) << endl;

  /**
   * @brief 結果の出力
   */

  auto reference_pose = result.at(Symbol('x', 0)).cast<Pose3>();
  auto current_pose = result.at(Symbol('x', 1)).cast<Pose3>();
  reference_frame_ptr->SetCameraPose(Pose_t(reference_pose.matrix()));
  current_frame_ptr->SetCameraPose(Pose_t(current_pose.matrix()));
  for (auto& [id, pos] : estimated_landmark_position) {
    auto lm_pos = result.at(Symbol('l', id)).cast<Point3>();
    pos = lm_pos;
  }

  return true;
}
}  // namespace vslam::initialization