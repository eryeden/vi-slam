#pragma once

#include <iostream>
#include <vector>
#include <map>
#include <random>
#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>

#include "feature_in_frame.hpp"
#include "frame.hpp"
#include "landmark.hpp"
#include "type_defines.hpp"

namespace initializer {

    namespace utils {
        /**
         * @brief 特徴点位置の初期化を行う
         * @details
         * features_currentのカメラ位置を座標系の原点、直交基底として特徴点の位置を推定する。
         * 特徴点の対応が取れたものはMaskに1が入る
         *
         * @param features_reference
         * @param features_current
         * @return dense_feature::feature_in_frame
         */
        double initialize_feature_points(const dense_feature::feature_in_frame &features_reference,
                                         const dense_feature::feature_in_frame &features_current,
                                         dense_feature::feature_in_frame &features_output);


        /**
         * @brief 特徴点位置とCurrentFrameの位置・姿勢を出力する
         * @details
         * 特徴点位置、CurrentFrameのPoseは、ReferenceFrameの座標系に出力する
         * 特徴点の対応が取れたものはIsOutlierがFalseになる。
         * @param frame_reference
         * @param frame_current
         * @param initialized_landmarks
         * @param current_frame_position_in_world
         * @param current_frame_attitude_in_world
         * @return
         */
        double initialize_feature_points(const vislam::data::frame &frame_reference,
                                         const vislam::data::frame &frame_current,
                                         std::vector<vislam::data::landmark> &initialized_landmarks,
                                         vislam::Vec3_t &current_frame_position_in_world,
                                         vislam::Quat_t &current_frame_attitude_in_world);

        double initialize_feature_points(const vislam::data::frame &frame_reference,
                                         const vislam::data::frame &frame_current,
                                         std::vector<vislam::data::landmark> &initialized_landmarks);


        /**
         * @brief PnPによるカメラPose推定
         */
        double estimate_frame_pose_pnp(const vislam::data::frame & frame_current,
                                       const std::unordered_map<uint64_t, vislam::data::landmark> & database_landmark,
                                       vislam::Vec3_t &current_frame_position_in_world,
                                       vislam::Quat_t &current_frame_attitude_in_world);


    } // namespace utils

    class initializer {
    public:
    private:
    };

} // namespace initializer