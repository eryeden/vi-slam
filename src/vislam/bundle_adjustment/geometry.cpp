//
// Created by anudev on 2020/02/19.
//

#include "geometry.hpp"

using namespace vislam;

geometry::utility::utility() = default;

Vec2_t geometry::utility::get_reprojection_error(const vislam::Vec2_t &landmark_position_in_device,
                                                 const Vec3_t &landmark_position_in_world,
                                                 const Mat33_t &camera_attitude_in_world,
                                                 const Vec3_t &camera_position_in_world,
                                                 const Mat33_t &camera_intrinsic_parameter) {
    Mat34_t translate_matrix_I_t;
    translate_matrix_I_t << Mat33_t::Identity(), camera_position_in_world; //! (I | t)を生成している
    Vec4_t extended_landmark_position_in_world;
    extended_landmark_position_in_world << landmark_position_in_world, 1.0;
    Mat34_t projection_matrix = camera_intrinsic_parameter * camera_attitude_in_world * translate_matrix_I_t;
    Vec3_t reprojected_vec = projection_matrix * extended_landmark_position_in_world;
    Vec2_t reprojected_position(reprojected_vec[0] / reprojected_vec[2], reprojected_vec[1] / reprojected_vec[2]);
    Vec2_t reprojection_error = reprojected_position - landmark_position_in_device;

    return reprojection_error;
}

MatRC_t<2, 3> geometry::utility::get_der_F_der_omega(const vislam::Vec3_t &landmark_position_in_world,
                                                     const vislam::Mat33_t &camera_attitude_in_world,
                                                     const vislam::Vec3_t &camera_position_in_world,
                                                     const vislam::Mat33_t &camera_intrinsic_parameter) {

    /**
     * @brief 再投影位置を計算する。(p/r, q/r)
     */
    Mat34_t translate_matrix_I_t;
    translate_matrix_I_t << Mat33_t::Identity(), camera_position_in_world; //! (I | t)を生成している
    Vec4_t extended_landmark_position_in_world;
    extended_landmark_position_in_world << landmark_position_in_world, 1.0;
    Mat34_t projection_matrix = camera_intrinsic_parameter * camera_attitude_in_world * translate_matrix_I_t;
    Vec3_t reprojected_vec = projection_matrix * extended_landmark_position_in_world;
    double p = reprojected_vec[0], q = reprojected_vec[1], r = reprojected_vec[2]; //! 式の表記に合わせる。

    /**
     * @brief p,q,rに対する角速度ベクトルの偏微分を計算する
     */
    Vec3_t der_p_der_omega, der_q_der_omega, der_r_der_omega;
    Vec3_t r1, r2, r3;
    double fx, fy, u0, v0;

    fx = camera_intrinsic_parameter(0, 0);
    fy = camera_intrinsic_parameter(1, 1);
    u0 = camera_intrinsic_parameter(0, 2);
    v0 = camera_intrinsic_parameter(1, 2);

    r1 << camera_attitude_in_world(0, 0), camera_attitude_in_world(1, 0), camera_attitude_in_world(2, 0);
    r2 << camera_attitude_in_world(0, 1), camera_attitude_in_world(1, 1), camera_attitude_in_world(2, 1);
    r3 << camera_attitude_in_world(0, 2), camera_attitude_in_world(1, 2), camera_attitude_in_world(2, 2);

    der_p_der_omega = (fx * r1 + u0 * r3).cross(landmark_position_in_world - camera_position_in_world);
    der_q_der_omega = (fy * r2 + v0 * r3).cross(landmark_position_in_world - camera_position_in_world);
    der_r_der_omega = r3.cross(landmark_position_in_world - camera_position_in_world);

    /**
     * @brief der_f_der_omegaは2x3の行列、上パートと下パートを別に計算して合体する方針をとっている
     */
    Vec3_t upper_der_f_der_omega, lower_der_f_der_omega;
    double epsilon = 1e-5;
    upper_der_f_der_omega = 1.0 / (r * r + epsilon) * (r * der_p_der_omega - p * der_r_der_omega);
    lower_der_f_der_omega = 1.0 / (r * r + epsilon) * (r * der_q_der_omega - q * der_r_der_omega);
    //! ここで上としたを合体する
    MatRC_t<2, 3> der_f_der_omega;
    der_f_der_omega << upper_der_f_der_omega, lower_der_f_der_omega;

    return der_f_der_omega;
}

MatRC_t<2, 3> geometry::utility::get_der_F_der_t(const vislam::Vec3_t &landmark_position_in_world,
                                                 const vislam::Mat33_t &camera_attitude_in_world,
                                                 const vislam::Vec3_t &camera_position_in_world,
                                                 const vislam::Mat33_t &camera_intrinsic_parameter) {

    /**
     * @brief 再投影位置を計算する。(p/r, q/r)
     */
    Mat34_t translate_matrix_I_t;
    translate_matrix_I_t << Mat33_t::Identity(), camera_position_in_world; //! (I | t)を生成している
    Vec4_t extended_landmark_position_in_world;
    extended_landmark_position_in_world << landmark_position_in_world, 1.0;
    Mat34_t projection_matrix = camera_intrinsic_parameter * camera_attitude_in_world * translate_matrix_I_t;
    Vec3_t reprojected_vec = projection_matrix * extended_landmark_position_in_world;
    double p = reprojected_vec[0], q = reprojected_vec[1], r = reprojected_vec[2]; //! 式の表記に合わせる。

    /**
     * @brief p,q,rに対するカメラ位置ベクトルの偏微分を計算する
     */
    Vec3_t der_p_der_t, der_q_der_t, der_r_der_t;
    Vec3_t r1, r2, r3;
    double fx, fy, u0, v0;

    fx = camera_intrinsic_parameter(0, 0);
    fy = camera_intrinsic_parameter(1, 1);
    u0 = camera_intrinsic_parameter(0, 2);
    v0 = camera_intrinsic_parameter(1, 2);

    r1 << camera_attitude_in_world(0, 0), camera_attitude_in_world(1, 0), camera_attitude_in_world(2, 0);
    r2 << camera_attitude_in_world(0, 1), camera_attitude_in_world(1, 1), camera_attitude_in_world(2, 1);
    r3 << camera_attitude_in_world(0, 2), camera_attitude_in_world(1, 2), camera_attitude_in_world(2, 2);

    der_p_der_t = -(fx * r1 + u0 * r3);
    der_q_der_t = -(fy * r2 + v0 * r3);
    der_r_der_t = -r3;

    /**
     * @brief der_f_der_tは2x3の行列、上パートと下パートを別に計算して合体する方針をとっている
     */
    Vec3_t upper_der_f_der_t, lower_der_f_der_t;
    double epsilon = 1e-5;
    upper_der_f_der_t = 1.0 / (r * r + epsilon) * (r * der_p_der_t - p * der_r_der_t);
    lower_der_f_der_t = 1.0 / (r * r + epsilon) * (r * der_q_der_t - q * der_r_der_t);
    //! ここで上としたを合体する
    MatRC_t<2, 3> der_f_der_t;
    der_f_der_t << upper_der_f_der_t, lower_der_f_der_t;

    return der_f_der_t;
}

MatRC_t<2, 3> geometry::utility::get_der_F_der_p(const vislam::Vec3_t &landmark_position_in_world,
                                                 const vislam::Mat33_t &camera_attitude_in_world,
                                                 const vislam::Vec3_t &camera_position_in_world,
                                                 const vislam::Mat33_t &camera_intrinsic_parameter) {

    /**
     * @brief 再投影位置を計算する。(p/r, q/r)
     */
    Mat34_t translate_matrix_I_t;
    translate_matrix_I_t << Mat33_t::Identity(), camera_position_in_world; //! (I | t)を生成している
    Vec4_t extended_landmark_position_in_world;
    extended_landmark_position_in_world << landmark_position_in_world, 1.0;
    Mat34_t projection_matrix = camera_intrinsic_parameter * camera_attitude_in_world * translate_matrix_I_t;
    Vec3_t reprojected_vec = projection_matrix * extended_landmark_position_in_world;
    double p = reprojected_vec[0], q = reprojected_vec[1], r = reprojected_vec[2]; //! 式の表記に合わせる。

    /**
     * @brief p,q,rに対するLandmark位置ベクトルの偏微分を計算する
     */
    Vec3_t der_p_der_p, der_q_der_p, der_r_der_p;
    der_p_der_p << projection_matrix(0, 0), projection_matrix(0, 1), projection_matrix(0, 2);
    der_q_der_p << projection_matrix(1, 0), projection_matrix(1, 1), projection_matrix(1, 2);
    der_r_der_p << projection_matrix(2, 0), projection_matrix(2, 1), projection_matrix(2, 2);

    /**
     * @brief der_f_der_pは2x3の行列、上パートと下パートを別に計算して合体する方針をとっている
     */
    Vec3_t upper_der_f_der_p, lower_der_f_der_p;
    double epsilon = 1e-5;
    upper_der_f_der_p = 1.0 / (r * r + epsilon) * (r * der_p_der_p - p * der_r_der_p);
    lower_der_f_der_p = 1.0 / (r * r + epsilon) * (r * der_q_der_p - q * der_r_der_p);
    //! ここで上としたを合体する
    MatRC_t<2, 3> der_f_der_p;
    der_f_der_p << upper_der_f_der_p, lower_der_f_der_p;

    return der_f_der_p;
}
