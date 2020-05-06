//
// Created by ery on 2020/05/05.
//

#pragma once

#include <Eigen/Eigen>
/**
 * @note
 * opengvはどうやらEigenのIncludeをしていない。
 * OpenGVをIncludeするまえにEigenを読み込む必要あり。
 */
#include <opengv/relative_pose/CentralRelativeAdapter.hpp>
#include <opengv/relative_pose/methods.hpp>
#include <opengv/sac/Ransac.hpp>
#include <opengv/sac_problems/relative_pose/CentralRelativePoseSacProblem.hpp>

#include "Frame.hpp"

namespace vslam::verification {

class FeatureVerification5PointRANSAC {
 public:
  FeatureVerification5PointRANSAC(double ransac_threshold_angle_rad,
                                  int32_t ransac_max_iterations,
                                  double ransac_probability);

  data::Frame RemoveOutlier(const data::Frame& frame_reference,
                            const data::Frame& frame_current);

 private:
  // OpenGV
  using ProblemMono =
      opengv::sac_problems::relative_pose::CentralRelativePoseSacProblem;
  using AdapterMono = opengv::relative_pose::CentralRelativeAdapter;
  opengv::sac::Ransac<ProblemMono> mono_ransac_;
};

}  // namespace vslam::verification