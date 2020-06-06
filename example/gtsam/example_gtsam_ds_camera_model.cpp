//
// Created by ery on 2020/05/31.
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

#include <utility>
#include <vector>

#include "Camera.hpp"
#include "ViewerViz.hpp"

using namespace std;
using namespace gtsam;

/**
 * @brief Define new GTSAM factor
 **/
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <boost/optional.hpp>

template <class POSE, class LANDMARK>
class DoubleSphereProjectionFactor : public NoiseModelFactor2<POSE, LANDMARK> {
 protected:
  // Keep a copy of measurement and calibration for I/O
  Point2 measured_;  ///< 2D measurement
  boost::shared_ptr<vslam::data::DoubleSphereCameraModel>
      K_;  ///< shared pointer to calibration object
  boost::optional<POSE>
      body_P_sensor_;  ///< The pose of the sensor in the body frame

  // verbosity handling for Cheirality Exceptions
  bool throwCheirality_;  ///< If true, rethrows Cheirality exceptions (default:
                          ///< false)
  bool verboseCheirality_;  ///< If true, prints text for Cheirality exceptions
                            ///< (default: false)

 public:
  /// shorthand for base class type
  typedef NoiseModelFactor2<POSE, LANDMARK> Base;

  /// shorthand for this class
  typedef DoubleSphereProjectionFactor<POSE, LANDMARK> This;

  /// shorthand for a smart pointer to a factor
  typedef boost::shared_ptr<This> shared_ptr;

  /// Default constructor
  DoubleSphereProjectionFactor()
      : measured_(0, 0), throwCheirality_(false), verboseCheirality_(false) {}

  /**
   * Constructor
   * TODO: Mark argument order standard (keys, measurement, parameters)
   * @param measured is the 2 dimensional location of point in image (the
   * measurement)
   * @param model is the standard deviation
   * @param poseKey is the index of the camera
   * @param pointKey is the index of the landmark
   * @param K shared pointer to the constant calibration
   * @param body_P_sensor is the transform from body to sensor frame (default
   * identity)
   */
  DoubleSphereProjectionFactor(
      const Point2& measured,
      const SharedNoiseModel& model,
      Key poseKey,
      Key pointKey,
      vslam::data::DoubleSphereCameraModel K,
      boost::optional<POSE> body_P_sensor = boost::none)
      : Base(model, poseKey, pointKey),
        measured_(measured),
        K_(K.Clone()),
        body_P_sensor_(body_P_sensor),
        throwCheirality_(false),
        verboseCheirality_(false) {}

  /**
   * Constructor with exception-handling flags
   * TODO: Mark argument order standard (keys, measurement, parameters)
   * @param measured is the 2 dimensional location of point in image (the
   * measurement)
   * @param model is the standard deviation
   * @param poseKey is the index of the camera
   * @param pointKey is the index of the landmark
   * @param K shared pointer to the constant calibration
   * @param throwCheirality determines whether Cheirality exceptions are
   * rethrown
   * @param verboseCheirality determines whether exceptions are printed for
   * Cheirality
   * @param body_P_sensor is the transform from body to sensor frame  (default
   * identity)
   */
  DoubleSphereProjectionFactor(
      const Point2& measured,
      const SharedNoiseModel& model,
      Key poseKey,
      Key pointKey,
      const boost::shared_ptr<vslam::data::DoubleSphereCameraModel>& K,
      bool throwCheirality,
      bool verboseCheirality,
      boost::optional<POSE> body_P_sensor = boost::none)
      : Base(model, poseKey, pointKey),
        measured_(measured),
        K_(K),
        body_P_sensor_(body_P_sensor),
        throwCheirality_(throwCheirality),
        verboseCheirality_(verboseCheirality) {}

  /** Virtual destructor */
  virtual ~DoubleSphereProjectionFactor() {}

  /// @return a deep copy of this factor
  virtual gtsam::NonlinearFactor::shared_ptr clone() const {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

  /**
   * print
   * @param s optional string naming the factor
   * @param keyFormatter optional formatter useful for printing Symbols
   */
  void print(const std::string& s = "",
             const KeyFormatter& keyFormatter = DefaultKeyFormatter) const {
    std::cout << s << "GenericProjectionFactor, z = ";
    traits<Point2>::Print(measured_);
    if (this->body_P_sensor_)
      this->body_P_sensor_->print("  sensor pose in body frame: ");
    Base::print("", keyFormatter);
  }

  /// equals
  virtual bool equals(const NonlinearFactor& p, double tol = 1e-9) const {
    const This* e = dynamic_cast<const This*>(&p);
    return e && Base::equals(p, tol) &&
           traits<Point2>::Equals(this->measured_, e->measured_, tol)
           //&& this->K_->equals(*e->K_, tol)
           && ((!body_P_sensor_ && !e->body_P_sensor_) ||
               (body_P_sensor_ && e->body_P_sensor_ &&
                body_P_sensor_->equals(*e->body_P_sensor_)));
  }

  /// Evaluate error h(x)-z and optionally derivatives
  Vector evaluateError(const Pose3& pose,
                       const Point3& point,
                       boost::optional<Matrix&> H1 = boost::none,
                       boost::optional<Matrix&> H2 = boost::none) const {
    try {
      //      if(body_P_sensor_) {
      //        if(H1) {
      //          gtsam::Matrix H0;
      //          PinholeCamera<CALIBRATION>
      //          camera(pose.compose(*body_P_sensor_, H0), *K_); Point2
      //          reprojectionError(camera.project(point, H1, H2, boost::none) -
      //          measured_); *H1 = *H1 * H0; return reprojectionError;
      //        } else {
      //          PinholeCamera<CALIBRATION>
      //          camera(pose.compose(*body_P_sensor_), *K_); return
      //          camera.project(point, H1, H2, boost::none) - measured_;
      //        }
      //      } else {
      //        PinholeCamera<CALIBRATION> camera(pose, *K_);
      //        return camera.project(point, H1, H2, boost::none) - measured_;
      //      }

      // Compute reprojection error and jacobian
      Matrix J_pose, J_point;
      Point3 p_camera_frame = pose.transformTo(point, J_pose, J_point);
      vslam::MatRC_t<2, 3> J_projection;
      vslam::Vec2_t project_point = K_->Project(p_camera_frame, J_projection);

      if (H1) {
        *H1 = J_projection * J_pose;
      }
      if (H2) {
        *H2 = J_projection * J_point;
      }
      return project_point - measured_;

    } catch (CheiralityException& e) {
      if (H1) *H1 = Matrix::Zero(2, 6);
      if (H2) *H2 = Matrix::Zero(2, 3);
      if (verboseCheirality_)
        std::cout << e.what() << ": Landmark "
                  << DefaultKeyFormatter(this->key2())
                  << " moved behind camera "
                  << DefaultKeyFormatter(this->key1()) << std::endl;
      if (throwCheirality_) throw CheiralityException(this->key2());
    }
    return Vector2::Constant(2.0 * K_->fx_);
  }

  /** return the measurement */
  const Point2& measured() const { return measured_; }

  /** return the calibration object */
  inline const boost::shared_ptr<vslam::data::DoubleSphereCameraModel>
  calibration() const {
    return K_;
  }

  /** return verbosity */
  inline bool verboseCheirality() const { return verboseCheirality_; }

  /** return flag for throwing cheirality exceptions */
  inline bool throwCheirality() const { return throwCheirality_; }

 private:
  /// Serialization function
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE& ar, const unsigned int /*version*/) {
    ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
    ar& BOOST_SERIALIZATION_NVP(measured_);
    // ar & BOOST_SERIALIZATION_NVP(K_);
    ar& BOOST_SERIALIZATION_NVP(body_P_sensor_);
    ar& BOOST_SERIALIZATION_NVP(throwCheirality_);
    ar& BOOST_SERIALIZATION_NVP(verboseCheirality_);
  }

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

int main(int argc, char* argv[]) {
  /**
   * @brief DSカメラモデルの準備
   */
  vslam::data::DoubleSphereCameraModel double_sphere_camera_model(
      0,
      512,
      512,
      30,
      157.59754538879774,
      157.5796622580977,
      254.99794156627608,
      256.87518673272509,
      -0.17520352912129096,
      0.5924447685008596);

  /**
   * @brief 観測モデルの定義
   * @note
   * Define the camera observation noise model
   * Isotropic error model
   * は円形の分布となっている。普通の二次元分布のように楕円状の分布ではない。
   */
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
  //! 観測値の生成
  vector<pair<Pose3, vector<Point2>>> measurements;
  for (const auto& pose : poses) {
    vector<Point2> current_measurements;
    vslam::EigenAllocatedVector<vslam::Vec3_t> pc_source;
    for (const auto& point : points) {
      // project
      current_measurements.emplace_back(
          double_sphere_camera_model.Project(pose.transformTo(point)));
    }
    measurements.emplace_back(pose, current_measurements);
  }
  //@}

  // Create a factor graph
  NonlinearFactorGraph graph;

  // Add a prior on pose x1. This indirectly specifies where the origin is.
  noiseModel::Diagonal::shared_ptr poseNoise = noiseModel::Diagonal::Sigmas(
      (Vector(6) << Vector3::Constant(0.1), Vector3::Constant(0.3))
          .finished());  // 30cm std on x,y,z 0.1 rad on roll,pitch,yaw
  graph.emplace_shared<PriorFactor<Pose3>>(
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

  for (size_t i = 0; i < measurements.size(); i++) {
    const auto& current_pose = measurements[i].first;
    const auto& current_observations = measurements[i].second;
    for (size_t j = 0; j < current_observations.size(); j++) {
      graph.emplace_shared<DoubleSphereProjectionFactor<Pose3, Point3>>(
          current_observations[j],
          measurementNoise,
          Symbol('x', i),
          Symbol('l', j),
          double_sphere_camera_model);
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
  graph.emplace_shared<PriorFactor<Point3>>(
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