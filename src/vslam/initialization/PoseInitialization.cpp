//
// Created by ery on 2020/06/03.
//

#include "PoseInitialization.hpp"

#include <Eigen/Eigen>
#include <opengv/absolute_pose/CentralAbsoluteAdapter.hpp>
#include <opengv/absolute_pose/methods.hpp>
#include <opengv/sac/Ransac.hpp>
#include <opengv/sac_problems/absolute_pose/AbsolutePoseSacProblem.hpp>

using namespace vslam;
using namespace vslam::data;
using namespace vslam::initialization;
using namespace opengv;

std::optional<Pose_t> vslam::initialization::InitializePose(
    const vslam::data::FrameWeakPtr& input_frame,
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
  ransac.threshold_ = 1.0 - cos(0.01 * M_PI / 180.0);
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
