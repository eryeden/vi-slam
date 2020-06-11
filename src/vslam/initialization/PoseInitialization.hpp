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
    const Pose_t& previous_frame_pose);

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
    bool use_previous_pose_factor = true);

}  // namespace vslam::initialization