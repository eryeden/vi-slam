//
// Created by ery on 2020/05/17.
//

#include <fmt/format.h>
#include <spdlog/spdlog.h>

// We will use Pose2 variables (x, y, theta) to represent the robot positions
#include <gtsam/geometry/Pose2.h>

// In GTSAM, measurement functions are represented as 'factors'. Several common
// factors have been provided with the library for solving robotics/SLAM/Bundle
// Adjustment problems. Here we will use Between factors for the relative motion
// described by odometry measurements. Also, we will initialize the robot at the
// origin using a Prior factor.
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

// When the factors are created, we will add them to a Factor Graph. As the
// factors we are using are nonlinear factors, we will need a Nonlinear Factor
// Graph.
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

// Finally, once all of the factors have been added to our factor graph, we will
// want to solve/optimize to graph to find the best (Maximum A Posteriori) set
// of variable values. GTSAM includes several nonlinear optimizers to perform
// this step. Here we will use the Levenberg-Marquardt solver
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

// Once the optimized values have been calculated, we can also calculate the
// marginal covariance of desired variables
#include <gtsam/nonlinear/Marginals.h>

// The nonlinear solvers within GTSAM are iterative solvers, meaning they
// linearize the nonlinear functions around an initial linearization point, then
// solve the linear system to update the linearization point. This happens
// repeatedly until the solver converges to a consistent set of variable values.
// This requires us to specify an initial guess for each variable, held in a
// Values container.
#include <gtsam/nonlinear/Values.h>

#include "ViewerViz.hpp"

/**
 * @brief A first example : Odometry
 * @details
 * 基本やることは、Factorとして、観測値の設定を行うこと。
 * Prior Factorとして、起点となる位置の情報を与えること。
 * 推定値の初期値を設定すること。
 * 最後に最適化関数を呼んで、最大事後確率となる推定値を探索すること。
 *
 * 追加で、今回問題の対象としている確率変数それぞれについて周辺化を実施して、
 * 共分散を求めること。
 */

using namespace std;
using namespace gtsam;

int main() {
  // Create an empty nonlinear factor graph
  NonlinearFactorGraph graph;

  // Add a prior on the first pose, setting it to the origin
  // A prior factor consists of a mean and a noise model (covariance matrix)
  Pose2 prior_mean(0.0, 0.0, 0.0);  // prior at origin
  auto prior_noise = noiseModel::Diagonal::Sigmas(Vector3(0.3, 0.3, 0.1));
  graph.add(PriorFactor<Pose2>(1, prior_mean, prior_noise));

  // Add odometry factors
  Pose2 odometry(2.0, 0.0, 30.0 * M_PI / 180.0);
  // For simplicity, we will use the same noise model for each odometry factor
  auto odometry_noise = noiseModel::Diagonal::Sigmas(Vector3(0.2, 0.2, 0.1));
  // Create odometry (Between) factors between consecutive poses
  graph.add(BetweenFactor<Pose2>(1, 2, odometry, odometry_noise));
  graph.add(BetweenFactor<Pose2>(2, 3, odometry, odometry_noise));

  graph.print("\nFactor Graph:\n");  // print

  // Create the data structure to hold the initialEstimate estimate to the
  // solution For illustrative purposes, these have been deliberately set to
  // incorrect values
  Values initial;
  initial.insert(1, Pose2(0.5, 0.0, 0.2));
  initial.insert(2, Pose2(2.3, 0.1, -0.2));
  initial.insert(3, Pose2(4.1, 0.1, 0.1));
  initial.print("\nInitial Estimate:\n");  // print

  // optimize using Levenberg-Marquardt optimization
  Values result = LevenbergMarquardtOptimizer(graph, initial).optimize();
  result.print("Final Result:\n");

  result.at(1).cast<Pose2>();

  // Calculate and print marginal covariances for all variables
  cout.precision(2);
  Marginals marginals(graph, result);
  cout << "x1 covariance:\n" << marginals.marginalCovariance(1) << endl;
  cout << "x2 covariance:\n" << marginals.marginalCovariance(2) << endl;
  cout << "x3 covariance:\n" << marginals.marginalCovariance(3) << endl;

  // Generate viewer
  auto viewer = vslam::viewer::ViewerViz();

  // Feed drawing primitives
  viewer.PushPrimitive(vslam::viewer::CoordinateSystemPrimitive(
      "world_origin", vslam::Vec3_t(0, 0, 0), vslam::Quat_t::Identity()));

  for (const auto& [id, res] : result) {
    auto pose = res.cast<Pose2>();
    viewer.PushPrimitive(vslam::viewer::CoordinateSystemPrimitive(
        "pose_" + std::to_string(id),
        {pose.x(), pose.y(), 0},
        vslam::Quat_t(
            Eigen::AngleAxisd(pose.theta(), vslam::Vec3_t(0.0, 0, 1.0))
                .toRotationMatrix())));

    vslam::Mat22_t tmp_cov;
    tmp_cov << marginals.marginalCovariance(id)(0, 0),
        marginals.marginalCovariance(id)(0, 1),
        marginals.marginalCovariance(id)(1, 0),
        marginals.marginalCovariance(id)(1, 1);
    viewer.PushPrimitive(
        vslam::viewer::Covariance2DPrimitive("cov_" + std::to_string(id),
                                             {pose.x(), pose.y()},
                                             pose.theta(),
                                             tmp_cov,
                                             {200, 1, 200}));
  }

  viewer.LaunchViewer(true);

  return 0;
}