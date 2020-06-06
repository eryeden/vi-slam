//
// Created by ery on 2020/05/31.
//

#pragma once

#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <boost/optional.hpp>

#include "Camera.hpp"
#include "type_defines.hpp"

namespace vslam::factor {

template <class POSE, class LANDMARK>
class DoubleSphereProjectionFactor
    : public gtsam::NoiseModelFactor2<POSE, LANDMARK> {
 protected:
  // Keep a copy of measurement and calibration for I/O
  gtsam::Point2 measured_;  ///< 2D measurement
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
  typedef gtsam::NoiseModelFactor2<POSE, LANDMARK> Base;

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
      const gtsam::Point2& measured,
      const gtsam::SharedNoiseModel& model,
      gtsam::Key poseKey,
      gtsam::Key pointKey,
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
      const gtsam::Point2& measured,
      const gtsam::SharedNoiseModel& model,
      gtsam::Key poseKey,
      gtsam::Key pointKey,
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
             const gtsam::KeyFormatter& keyFormatter =
                 gtsam::DefaultKeyFormatter) const {
    std::cout << s << "GenericProjectionFactor, z = ";
    gtsam::traits<gtsam::Point2>::Print(measured_);
    if (this->body_P_sensor_)
      this->body_P_sensor_->print("  sensor pose in body frame: ");
    Base::print("", keyFormatter);
  }

  /// equals
  virtual bool equals(const gtsam::NonlinearFactor& p,
                      double tol = 1e-9) const {
    const This* e = dynamic_cast<const This*>(&p);
    return e && Base::equals(p, tol) &&
           gtsam::traits<gtsam::Point2>::Equals(
               this->measured_, e->measured_, tol)
           //&& this->K_->equals(*e->K_, tol)
           && ((!body_P_sensor_ && !e->body_P_sensor_) ||
               (body_P_sensor_ && e->body_P_sensor_ &&
                body_P_sensor_->equals(*e->body_P_sensor_)));
  }

  /// Evaluate error h(x)-z and optionally derivatives
  gtsam::Vector evaluateError(
      const gtsam::Pose3& pose,
      const gtsam::Point3& point,
      boost::optional<gtsam::Matrix&> H1 = boost::none,
      boost::optional<gtsam::Matrix&> H2 = boost::none) const {
    try {
      if (body_P_sensor_) {
        if (H1) {
          gtsam::Matrix H0;
          auto sensor_pose = pose.compose(*body_P_sensor_, H0);

          // Compute reprojection error and jacobian
          gtsam::Matrix J_pose, J_point;
          gtsam::Point3 p_camera_frame =
              pose.transformTo(point, J_pose, J_point);
          vslam::MatRC_t<2, 3> J_projection;
          vslam::Vec2_t project_point =
              K_->Project(p_camera_frame, J_projection);

          if (H1) {
            *H1 = J_projection * J_pose * H0;
          }
          if (H2) {
            *H2 = J_projection * J_point;
          }
          return project_point - measured_;

        } else {
          auto sensor_pose = pose.compose(*body_P_sensor_);
          gtsam::Point3 p_camera_frame = pose.transformTo(point);
          vslam::Vec2_t project_point = K_->Project(p_camera_frame);
          return project_point - measured_;
        }
      } else {
        // Compute reprojection error and jacobian
        gtsam::Matrix J_pose, J_point;
        gtsam::Point3 p_camera_frame = pose.transformTo(point, J_pose, J_point);
        vslam::MatRC_t<2, 3> J_projection;
        vslam::Vec2_t project_point = K_->Project(p_camera_frame, J_projection);

        if (H1) {
          *H1 = J_projection * J_pose;
        }
        if (H2) {
          *H2 = J_projection * J_point;
        }
        return project_point - measured_;
      }

    } catch (gtsam::CheiralityException& e) {
      //      if (H1) *H1 = gtsam::Matrix::Zero(2, 6);
      //      if (H2) *H2 = gtsam::Matrix::Zero(2, 3);
      //      if (verboseCheirality_)
      //        std::cout << e.what() << ": Landmark "
      //                  << DefaultKeyFormatter(this->key2())
      //                  << " moved behind camera "
      //                  << DefaultKeyFormatter(this->key1()) << std::endl;
      //      if (throwCheirality_) throw CheiralityException(this->key2());
    } catch (data::ProjectionErrorException& exception) {
      if (throwCheirality_) throw CheiralityException(this->key2());
    }
    return gtsam::Vector2::Constant(2.0 * K_->fx_);
  }

  /** return the measurement */
  const gtsam::Point2& measured() const { return measured_; }

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
    ar& BOOST_SERIALIZATION_NVP(K_);
    ar& BOOST_SERIALIZATION_NVP(body_P_sensor_);
    ar& BOOST_SERIALIZATION_NVP(throwCheirality_);
    ar& BOOST_SERIALIZATION_NVP(verboseCheirality_);
  }

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace vslam::factor