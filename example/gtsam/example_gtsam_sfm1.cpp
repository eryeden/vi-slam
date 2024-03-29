//
// Created by ery on 2020/05/20.
//

/**
 * @file    SFMExample.cpp
 * @brief   A structure-from-motion problem on a simulated dataset
 * @author  Duy-Nguyen Ta
 */

// For loading the data, see the comments therein for scenario (camera rotates
// around cube)
#include "SFMdata.h"

// Camera observations of landmarks (i.e. pixel coordinates) will be stored as
// Point2 (x, y).
#include <gtsam/geometry/Point2.h>

// Each variable in the system (poses and landmarks) must be identified with a
// unique key. We can either use simple integer keys (1, 2, 3, ...) or symbols
// (X1, X2, L1). Here we will use Symbols
#include <gtsam/inference/Symbol.h>

// In GTSAM, measurement functions are represented as 'factors'. Several common
// factors have been provided with the library for solving robotics/SLAM/Bundle
// Adjustment problems. Here we will use Projection factors to model the
// camera's landmark observations. Also, we will initialize the robot at some
// location using a Prior factor.
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/ProjectionFactor.h>

// When the factors are created, we will add them to a Factor Graph. As the
// factors we are using are nonlinear factors, we will need a Nonlinear Factor
// Graph.
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

// Finally, once all of the factors have been added to our factor graph, we will
// want to solve/optimize to graph to find the best (Maximum A Posteriori) set
// of variable values. GTSAM includes several nonlinear optimizers to perform
// this step. Here we will use a trust-region method known as Powell's Degleg
#include <gtsam/nonlinear/DoglegOptimizer.h>

// The nonlinear solvers within GTSAM are iterative solvers, meaning they
// linearize the nonlinear functions around an initial linearization point, then
// solve the linear system to update the linearization point. This happens
// repeatedly until the solver converges to a consistent set of variable values.
// This requires us to specify an initial guess for each variable, held in a
// Values container.
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>

#include <vector>

#include "ViewerViz.hpp"

using namespace std;
using namespace gtsam;

/* ************************************************************************* */
int main(int argc, char* argv[]) {
  /**
   * @brief カメラモデルの定義、sはSkewと言うらしい。
   * @details
   * [fx, s, ux; 0, fy, uy; 0,0,1]としてsが導入されるとのこと。
   * sはここでは0なので、何かなければ無視しておけば良さそう。
   */
  // Define the camera calibration parameters
  Cal3_S2::shared_ptr K(new Cal3_S2(50.0, 50.0, 0.0, 50.0, 50.0));

  // Define the camera observation noise model
  // Isotropic error model
  // は円形の分布となっている。普通の二次元分布のように楕円状の分布ではない。
  noiseModel::Isotropic::shared_ptr measurementNoise =
      noiseModel::Isotropic::Sigma(2, 1.0);  // one pixel in u and v

  /**
   * @brief Datasetを生成
   * @details
   * ３Dのランドマーク位置と、カメラの観測Poseを生成
   */
  //@{
  // Create the set of ground-truth landmarks
  vector<Point3> points = createPoints();
  // Create the set of ground-truth poses
  vector<Pose3> poses = createPoses();
  //@}

  // Create a factor graph
  NonlinearFactorGraph graph;

  // Add a prior on pose x1. This indirectly specifies where the origin is.
  noiseModel::Diagonal::shared_ptr poseNoise = noiseModel::Diagonal::Sigmas(
      (Vector(6) << Vector3::Constant(0.1), Vector3::Constant(0.3))
          .finished());  // 30cm std on x,y,z 0.1 rad on roll,pitch,yaw
  graph.emplace_shared<PriorFactor<Pose3> >(
      Symbol('x', 0), poses[0], poseNoise);  // add directly to graph

  /**
   * @brief ランドマークの観測Factorを追加する。
   * @details
   * GenericProjectionFactorにカメラの姿勢タイプPose3とランドマークの姿勢タイプPoint3、
   * カメラモデルCal3_S2を渡すことで、いろんな場合における最投影誤差を計算できるぽい。
   * 引数には、FrameIDとLandmarkID、カメラモデルのパラメータを入れているぽい。
   */
  // Simulated measurements from each camera pose, adding them to the factor
  // graph
  for (size_t i = 0; i < poses.size(); ++i) {
    PinholeCamera<Cal3_S2> camera(poses[i], *K);
    for (size_t j = 0; j < points.size(); ++j) {
      Point2 measurement = camera.project(points[j]);
      graph.emplace_shared<GenericProjectionFactor<Pose3, Point3, Cal3_S2> >(
          measurement, measurementNoise, Symbol('x', i), Symbol('l', j), K);
    }
  }

  /**
   * @brief スケール不定性を排除する
   * @details
   * カメラ位置X0の位置をPriorFactorで拘束したことに加えて、
   * LandmarkL0の位置もPriorFactorも初期位置に拘束することでスケールの不訂正を排除できる。
   */
  // Because the structure-from-motion problem has a scale ambiguity, the
  // problem is still under-constrained Here we add a prior on the position of
  // the first landmark. This fixes the scale by indicating the distance between
  // the first camera and the first landmark. All other landmark positions are
  // interpreted using this scale.
  noiseModel::Isotropic::shared_ptr pointNoise =
      noiseModel::Isotropic::Sigma(3, 0.1);
  graph.emplace_shared<PriorFactor<Point3> >(
      Symbol('l', 0), points[0], pointNoise);  // add directly to graph
  graph.print("Factor Graph:\n");

  // Create the data structure to hold the initial estimate to the solution
  // Intentionally initialize the variables off from the ground truth
  Values initialEstimate;
  for (size_t i = 0; i < poses.size(); ++i)
    initialEstimate.insert(
        Symbol('x', i),
        poses[i].compose(Pose3(Rot3::Rodrigues(-0.1, 0.2, 0.25),
                               Point3(0.05, -0.10, 0.20))));
  for (size_t j = 0; j < points.size(); ++j)
    initialEstimate.insert<Point3>(Symbol('l', j),
                                   points[j] + Point3(-0.25, 0.20, 0.15));
  initialEstimate.print("Initial Estimates:\n");

  /* Optimize the graph and print results */
  Values result = DoglegOptimizer(graph, initialEstimate).optimize();
  result.print("Final results:\n");
  cout << "initial error = " << graph.error(initialEstimate) << endl;
  cout << "final error = " << graph.error(result) << endl;

  // Get covariance
  Marginals marginals(graph, result);
  marginals.print("Output marginals.");

  // Generate viewer
  auto viewer = vslam::viewer::ViewerViz();
  // Feed drawing primitives
  viewer.PushPrimitive(vslam::viewer::CoordinateSystemPrimitive(
      "world_origin", vslam::Vec3_t(0, 0, 0), vslam::Quat_t::Identity()));

  vslam::EigenAllocatedVector<vslam::Vec3_t> pc_source;

  spdlog::info("######### RENDERING SECTION ##########");

  for (const auto& [id, res] : result) {
    Symbol symbol(id);
    Key key = symbol.chr();
    if (key == 'x') {
      spdlog::info("Marginals of x:{}", symbol.index());
      std::cout << marginals.marginalCovariance(symbol) << std::endl;
      vslam::Mat33_t cov_position =
          marginals.marginalCovariance(symbol).block<3, 3>(0, 0);

      auto pose = res.cast<Pose3>();
      viewer.PushPrimitive(vslam::viewer::CoordinateSystemPrimitive(
          "pose_" + std::to_string(id),
          {pose.x(), pose.y(), pose.z()},
          vslam::Quat_t(pose.rotation().matrix())));

      viewer.PushPrimitive(vslam::viewer::CovariancePrimitive(
          "cov_" + std::to_string(id),
          {pose.x(), pose.y(), pose.z()},
          vslam::Quat_t(pose.rotation().matrix()),
          cov_position,
          {200, 1, 200},
          0.5));

    } else if (key == 'l') {
      spdlog::info("Marginals of l:{}", symbol.index());
      std::cout << marginals.marginalCovariance(symbol) << std::endl;

      auto p = res.cast<Point3>();
      pc_source.emplace_back(p);
      viewer.PushPrimitive(vslam::viewer::CovariancePrimitive(
          "cov_" + std::to_string(id),
          p,
          vslam::Quat_t::Identity(),
          marginals.marginalCovariance(symbol),
          {200, 1, 200},
          0.5));
    }
  }

  vslam::viewer::PointCloudPrimitive pcp("pc", pc_source);
  viewer.PushPrimitive(pcp);

  // Create a window and render primitives.
  viewer.LaunchViewer(true);

  return 0;
}