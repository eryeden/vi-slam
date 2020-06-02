//
// Created by ery on 2020/06/01.
//

#pragma once

#include <spdlog/spdlog.h>

#include <Eigen/Eigen>
#include <opengv/relative_pose/CentralRelativeAdapter.hpp>
#include <opengv/relative_pose/methods.hpp>
#include <opengv/sac/Ransac.hpp>
#include <opengv/sac_problems/relative_pose/CentralRelativePoseSacProblem.hpp>
#include <opengv/triangulation/methods.hpp>

#include "Frame.hpp"
#include "type_defines.hpp"

namespace vslam::initialization {

/**
 * @brief 2枚のFrameから、相対的Pose、特徴点位置の情報を推定する
 * @param reference_frame[in] :
 * 基準Frame、出力するPoseは基準FrameをIとして出力する
 * @param current_frame[in] : 現在Frame
 * @param current_frame_pose[out] : 推定したCurrent frameのPose
 * @param estimated_landmark_position[out] : 基準Frameの座標系でのLandmark
 * position
 * @return : Current frame Poseの推定と、Landmark位置推定に成功したか？
 */
bool TryInitialize(const data::FrameWeakPtr& reference_frame,
                   const data::FrameWeakPtr& current_frame,
                   Pose_t& current_frame_pose,
                   vslam::EigenAllocatedUnorderedMap<database_index_t, Vec3_t>&
                       estimated_landmark_position,
                   double inlier_rate_threshold,
                   double parallax_threshold);

/**
 * @brief 最適化を使って、初期化の結果をRefineする
 * @param reference_frame
 * @param current_frame
 * @param current_frame_pose
 * @param estimated_landmark_position
 * @return
 */
bool RefineInitializedMap(
    const data::FrameWeakPtr& reference_frame,
    const data::FrameWeakPtr& current_frame,
    vslam::EigenAllocatedUnorderedMap<database_index_t, Vec3_t>&
        estimated_landmark_position);

}  // namespace vslam::initialization