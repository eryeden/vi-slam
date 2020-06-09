//
// Created by ery on 2020/06/09.
//

#pragma once

#include <basalt/image/image_pyr.h>
#include <tbb/blocked_range.h>
#include <tbb/concurrent_unordered_map.h>
#include <tbb/parallel_for.h>

#include <sophus/se2.hpp>

#include "FrontendParameters.hpp"
#include "optical_flow.h"
#include "patch.h"
#include "spdlog/spdlog.h"
#include "utils/keypoints.h"

namespace vslam::feature {

class FrameToFrameOpticalFlow {
 public:
  using Scalar = float;  // 内部でAffine compact 2f
                         // を利用しているのでFloat以外ではコンパイル不可
  using Pattern = basalt::Pattern24<Scalar>;

  typedef basalt::OpticalFlowPatch<Scalar, Pattern> PatchT;

  typedef Eigen::Matrix<Scalar, 2, 1> Vector2;
  typedef Eigen::Matrix<Scalar, 2, 2> Matrix2;

  typedef Eigen::Matrix<Scalar, 3, 1> Vector3;
  typedef Eigen::Matrix<Scalar, 3, 3> Matrix3;

  typedef Eigen::Matrix<Scalar, 4, 1> Vector4;
  typedef Eigen::Matrix<Scalar, 4, 4> Matrix4;

  typedef Sophus::SE2<Scalar> SE2;

  //  FrameToFrameOpticalFlow(const FrontendParameter& frontend_parameter,
  //                          const basalt::Calibration<double>& calib);
  //  ~FrameToFrameOpticalFlow() = default;

  //  std::optional<basalt::OpticalFlowResult::Ptr> ProcessFrame(int64_t
  //  curr_t_ns, basalt::OpticalFlowInput::Ptr& new_img_vec);

  /**
   * @brief
   * 前回のImagePyramidにおける特徴点から、今回のImagePyramidの特徴点位置を推定する。
   * @details Recover trackingによるOutlier排除も実施!!!
   * @param pyr_1[in] : Previous image pyramid
   * @param pyr_2[in] : Current image pyramid
   * @param transform_map_1[in] : Previous feature points
   * @param transform_map_2[out] : Estimated feature points
   */
  static void TrackPoints(
      const basalt::ManagedImagePyr<u_int16_t>& pyr_1,
      const basalt::ManagedImagePyr<u_int16_t>& pyr_2,
      const Eigen::aligned_map<basalt::KeypointId, Eigen::AffineCompact2f>&
          transform_map_1,
      Eigen::aligned_map<basalt::KeypointId, Eigen::AffineCompact2f>&
          transform_map_2,
      double optical_flow_max_recovered_distance,
      double optical_flow_levels,
      double optical_flow_max_iterations);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 private:
  static bool TrackPoint(const basalt::ManagedImagePyr<uint16_t>& old_pyr,
                         const basalt::ManagedImagePyr<uint16_t>& pyr,
                         const Eigen::AffineCompact2f& old_transform,
                         Eigen::AffineCompact2f& transform,
                         double optical_flow_levels,
                         double optical_flow_max_iterations);
  static bool TrackPointAtLevel(const basalt::Image<const u_int16_t>& img_2,
                                const PatchT& dp,
                                Eigen::AffineCompact2f& transform,
                                double optical_flow_max_iterations);
};

}  // namespace vslam::feature