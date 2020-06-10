//
// Created by ery on 2020/06/09.
//

#include "frame_to_frame_optical_flow_single_thread.hpp"

using namespace vslam;
using namespace vslam::feature;
using namespace basalt;

void FrameToFrameOpticalFlow::TrackPoints(
    const basalt::ManagedImagePyr<u_int16_t>& pyr_1,
    const basalt::ManagedImagePyr<u_int16_t>& pyr_2,
    const Eigen::aligned_map<KeypointId, Eigen::AffineCompact2f>&
        transform_map_1,
    Eigen::aligned_map<KeypointId, Eigen::AffineCompact2f>& transform_map_2,
    double optical_flow_max_recovered_distance,
    double optical_flow_levels,
    double optical_flow_max_iterations) {
  size_t num_points = transform_map_1.size();

  std::vector<KeypointId> ids;
  Eigen::aligned_vector<Eigen::AffineCompact2f> init_vec;

  ids.reserve(num_points);
  init_vec.reserve(num_points);

  for (const auto& kv : transform_map_1) {
    ids.push_back(kv.first);
    init_vec.push_back(kv.second);
  }

  tbb::concurrent_unordered_map<KeypointId,
                                Eigen::AffineCompact2f,
                                std::hash<KeypointId>>
      result;

  auto compute_func = [&](const tbb::blocked_range<size_t>& range) {
    for (size_t r = range.begin(); r != range.end(); ++r) {
      const KeypointId id = ids[r];

      const Eigen::AffineCompact2f& transform_1 = init_vec[r];
      Eigen::AffineCompact2f transform_2 = transform_1;

      bool valid = TrackPoint(pyr_1,
                              pyr_2,
                              transform_1,
                              transform_2,
                              optical_flow_levels,
                              optical_flow_max_iterations);

      if (valid) {
        Eigen::AffineCompact2f transform_1_recovered = transform_2;

        valid = TrackPoint(pyr_2,
                           pyr_1,
                           transform_2,
                           transform_1_recovered,
                           optical_flow_levels,
                           optical_flow_max_iterations);

        if (valid) {
          Scalar dist2 =
              (transform_1.translation() - transform_1_recovered.translation())
                  .squaredNorm();

          if (dist2 < optical_flow_max_recovered_distance) {
            result[id] = transform_2;
          }
        }
      }
    }
  };

  tbb::blocked_range<size_t> range(0, num_points);

  tbb::parallel_for(range, compute_func);
  // compute_func(range);

  transform_map_2.clear();
  transform_map_2.insert(result.begin(), result.end());
}

bool FrameToFrameOpticalFlow::TrackPoint(
    const basalt::ManagedImagePyr<uint16_t>& old_pyr,
    const basalt::ManagedImagePyr<uint16_t>& pyr,
    const Eigen::AffineCompact2f& old_transform,
    Eigen::AffineCompact2f& transform,
    double optical_flow_levels,
    double optical_flow_max_iterations) {
  bool patch_valid = true;

  transform.linear().setIdentity();

  for (int level = optical_flow_levels; level >= 0 && patch_valid; level--) {
    const Scalar scale = 1 << level;

    transform.translation() /= scale;

    PatchT p(old_pyr.lvl(level), old_transform.translation() / scale);

    // Perform tracking on current level
    patch_valid &= TrackPointAtLevel(
        pyr.lvl(level), p, transform, optical_flow_max_iterations);

    transform.translation() *= scale;
  }

  transform.linear() = old_transform.linear() * transform.linear();

  return patch_valid;
}

bool FrameToFrameOpticalFlow::TrackPointAtLevel(
    const basalt::Image<const u_int16_t>& img_2,
    const PatchT& dp,
    Eigen::AffineCompact2f& transform,
    double optical_flow_max_iterations) {
  bool patch_valid = true;

  for (int iteration = 0;
       patch_valid && iteration < optical_flow_max_iterations;
       iteration++) {
    typename PatchT::VectorP res;

    typename PatchT::Matrix2P transformed_pat =
        transform.linear().matrix() * PatchT::pattern2;
    transformed_pat.colwise() += transform.translation();

    bool valid = dp.residual(img_2, transformed_pat, res);

    if (valid) {
      Vector3 inc = -dp.H_se2_inv_J_se2_T * res;
      transform *= SE2::exp(inc).matrix();

      const int filter_margin = 2;

      if (!img_2.InBounds(transform.translation(), filter_margin))
        patch_valid = false;
    } else {
      patch_valid = false;
    }
  }

  return patch_valid;
}
