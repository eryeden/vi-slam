//
// Created by ery on 2020/06/03.
//

#pragma once

#include <optional>

#include "Frame.hpp"
#include "ThreadsafeContainer.hpp"
#include "type_defines.hpp"

namespace vslam::initialization {

/**
 * @brief 3Point baseの手法でFramePoseを推定する
 * @details
 * コレだけでTriangulationすることをはギルティー
 * @param input_frame
 * @param map_database
 * @return
 */
std::optional<Pose_t> InitializePose(
    data::FrameWeakPtr&& input_frame,
    std::shared_ptr<data::ThreadsafeMapDatabase>& map_database,
    const Pose_t& previous_frame_pose,
    double ransac_threshold,
    int32_t ransac_max_iterations,
    double ransac_probability);

/**
 * @brief Motion only BAを実施
 * @details
 * 結果がとても良くなるのてマジで必須
 * @param input_frame
 * @param map_database
 * @return
 */
std::optional<Pose_t> RefinePose(
    data::FrameWeakPtr&& input_frame,
    std::shared_ptr<data::ThreadsafeMapDatabase>& map_database,
    double reprojection_noise_sigma,
    double landmark_position_sigma,
    bool use_previous_pose_factor,
    double previous_position_sigma,
    double previous_orientation_sigma);

}  // namespace vslam::initialization