#pragma once

#include <iostream>
#include <vector>
#include <map>
#include <random>
#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>

namespace dense_feature
{

namespace utils
{
// 曲率画像を生成する
cv::Mat generate_curvature_image(const cv::Mat &input_mono);

// Local maxの探索を行う
cv::Point2i track_local_max(
    const cv::Mat &img_mono,
    const cv::Point2f &initial_point);
// 探索開始点をアンカーとしてLocal maxを探索する
cv::Point2i track_local_max_with_regularization(
    const cv::Mat &img_mono,
    const cv::Point2f &initial_point);

// 3近傍の最大値探索を行う
cv::Point2i get_neighbor_max(const cv::Mat &img_mono, const cv::Point2i &input_point);
// 3近傍の探索を推定点で正則化し探索する
cv::Point2i get_neighbor_max_with_regularization(
    const cv::Mat &img_mono,
    const cv::Point2i &input_point,
    const double lambda_coeff,
    const double sigma_coeff,
    const cv::Point2f &estimated_point);

//正則化項の計算
double get_regularization_term(
    const double lambda_coeff,
    const double sigma_coeff,
    const cv::Point2f &input_point,
    const cv::Point2f &estimated_point);

// Affine行列をもとに次フレームの特徴点位置を計算する
bool warp_point(
    const cv::Point2f &input,
    const cv::Mat &affine_mat,
    const cv::Size &image_size,
    cv::Point2f &output);

} // namespace utils

/**
 * @brief フレームふ含まれる特徴点を保存しておく。
 * @details
 * ## 特徴点の保存について
 * 特徴点にはIDが付与される。これが、別のstd::vectorに入れられていて、同インデックスのFeatureとIDが対応している。
 * 
 * 
 * 
 */
class feature_in_frame
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    feature_in_frame(const uint64_t frame_id,
                     const std::vector<uint64_t> &input_feature_ids,
                     const std::vector<Eigen::Vector2i, Eigen::aligned_allocator<Eigen::Vector2i>> &input_features);
    feature_in_frame();

    std::vector<Eigen::Vector2i, Eigen::aligned_allocator<Eigen::Vector2i>> features;
    std::vector<uint64_t> featureIDs;
    uint64_t frameID;

private:
};

/**
 * @brief VETAMIN-E baseの密な特徴点抽出とトラッキングを行う
 * @details
 * 
 * 
 */
class dense_feature_extructor
{

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    dense_feature_extructor(const double lambda_,
                            const double sigma_,
                            const double dominant_flow_scale_ = 1.0 / 2.0);

    /**
     * @brief 特徴点の検出とトラッキングをする
     * 
     * @param input_color 
     */
    void detect_and_track(const cv::Mat &input, bool is_color = true);

    /**
     * @brief 検出した特徴点をフレームごとに保存する
     * 
     */
    std::vector<feature_in_frame>
        features;
    std::vector<uint64_t> frameIDs;

private:
    /**
     * @brief 初期化するか？
     * 
     */
    bool is_initialize;

    /**
     * @brief トラッキングパラメータ
     * 
     */
    double lambda;
    double sigma;
    double dominant_flow_scale;

    // 大域的な画像のフローを求める
    cv::Mat get_dominant_flow(const cv::Mat &img_mono);
    // フロー計算用
    cv::Mat prev_descriptor;
    std::vector<cv::KeyPoint> prev_keypoints;
    bool is_initialize_dominant_flow;
    cv::Mat prev_img;

    /**
     * @brief 特徴点の初期化を行う
     * 
     * @param img_curvature 
     * @param previouse_frame 
     * @param num_points 
     * @return feature_in_frame 
     */
    feature_in_frame initialize_features(
        const cv::Mat &img_curvature,
        const feature_in_frame &previous_frame,
        const int32_t num_points = 10000);
};

} // namespace dense_feature