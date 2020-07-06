//
// Created by ery on 2020/06/19.
//
#pragma once

#include <gtsam/nonlinear/ISAM2.h>

#include "BackendBase.hpp"

namespace vslam::backend::utility {

/**
 * @brief
 * Landmarkの被観測情報を更新する。Landmarkが存在しない場合は新規登録し、被観測情報を追加する。
 * @param map_database
 * @param input_frame
 */
void RegisterLandmarkObservation(
    std::shared_ptr<data::ThreadsafeMapDatabase>& map_database,
    const data::FrameWeakPtr& input_frame);

bool InitializeISAM2(std::shared_ptr<gtsam::ISAM2>& isam_2,
                     std::shared_ptr<data::ThreadsafeMapDatabase>& map_database,
                     data::FrameWeakPtr&& reference_frame,
                     data::FrameWeakPtr&& current_frame,
                     double isam2_reprojection_noise_sigma,
                     double isam2_prior_pose_position_sigma,
                     double isam2_prior_pose_orientation_sigma,
                     int32_t isam2_iteration_number);

/**
 * @brief 2つのKeyFrame間でTriangulateする
 * @param map_database
 * @param current_key_frame
 * @param previous_key_frame
 * @param triangulated_landmarks
 * @param reprojection_error_threshold
 * @param minimum_parallax_threshold
 * @return
 */
bool TriangulateKeyFrame(
    std::shared_ptr<data::ThreadsafeMapDatabase>& map_database,
    const data::FrameWeakPtr& current_key_frame,
    const data::FrameWeakPtr& previous_key_frame,
    vslam::EigenAllocatedUnorderedMap<database_index_t,
                                      vslam::data::LandmarkWeakPtr>&
        triangulated_landmarks,
    double reprojection_error_threshold,
    double minimum_parallax_threshold);
/**
 * @brief
 * 新規観測Frameで観測された未初期化Landmarkのうちから、被観測Frameを探索、初期化を試みる
 * @param map_database
 * @param current_key_frame
 * @param triangulated_landmarks
 * @param reprojection_error_threshold
 * @param minimum_parallax_threshold
 * @return
 */
bool TriangulateKeyFrame(
    std::shared_ptr<data::ThreadsafeMapDatabase>& map_database,
    const data::FrameWeakPtr& current_key_frame,
    vslam::EigenAllocatedUnorderedMap<database_index_t,
                                      vslam::data::LandmarkWeakPtr>&
        triangulated_landmarks,
    double reprojection_error_threshold,
    double minimum_parallax_threshold,
    int32_t max_triangulated_number = -1);

bool UpdateISAMObservation(
    std::shared_ptr<gtsam::ISAM2>& isam_2,
    std::shared_ptr<data::ThreadsafeMapDatabase>& map_database,
    vslam::EigenAllocatedUnorderedMap<database_index_t,
                                      vslam::data::LandmarkWeakPtr>&
        triangulated_landmarks,
    double isam2_reprojection_noise_sigma,
    int32_t isam2_iteration_number,
    double reprojection_error_threshold);

}  // namespace vslam::backend::utility