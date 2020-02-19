//
// Created by anudev on 2020/02/19.
//

#pragma once

#include "camera.hpp"
#include "frame.hpp"
#include "landmark.hpp"

#include "Eigen/Dense"
#include "Eigen/Sparse"


namespace vislam::geometry {

/**
 * @brief 幾何的な計算について集めたクラス関係
 */
    class utility {

    public:

        utility();

        /**
         * @brief 再投影誤差を計算する。カメラモデルはピンホールのみサポート。
         * @param landmark_position_in_device
         * @param landmark_position_in_world
         * @param camera_attitude_in_world
         * @param camera_position_in_world
         * @param camera_intrinsic_parameter
         * @return
         */
        static Vec2_t get_reprojection_error(
                const Vec2_t &landmark_position_in_device,
                const Vec3_t &landmark_position_in_world,
                const Mat33_t &camera_attitude_in_world,
                const Vec3_t &camera_position_in_world,
                const Mat33_t &camera_intrinsic_parameter);

        static MatRC_t<2, 3> get_der_F_der_omega();

        static MatRC_t<2, 3> get_der_F_der_t();

        static MatRC_t<2, 3> get_der_F_der_p();


    };


}