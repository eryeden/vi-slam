#pragma once

#include <iostream>
#include <vector>
#include <map>
#include <random>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>

#include "log_util.h"

class dense_feature
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    dense_feature(uint64_t id_, const Eigen::Vector2i &feature_point);
    dense_feature();

    void add_feature(const Eigen::Vector2i &feature);
    const Eigen::Vector2i get_latest_feature() const;
    const std::vector<Eigen::Vector2i, Eigen::aligned_allocator<Eigen::Vector2i>> &get_feature_history() const;

    uint64_t get_id() const;
    void set_id(const uint64_t id_);

    bool get_is_tracking() const;
    void set_is_tracking(const bool is_tracking_);

private:
    uint64_t id;
    std::vector<Eigen::Vector2i, Eigen::aligned_allocator<Eigen::Vector2i>> feature_history;

    bool is_tracking;
};

class frame_dense_feature
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // using feature_and_id_pair = std::map<uint64_t, Eigen::Vector2i,
    //                                      std::less<uint64_t>,
    //                                      Eigen::aligned_allocator<std::pair<const uint64_t, Eigen::Vector2i>>>;

    using feature_and_id_pair = std::map<uint64_t, Eigen::Vector2i>;

    frame_dense_feature()
        : frame_id(0)
    {
    }

    frame_dense_feature(const uint64_t frame_id, const feature_and_id_pair &features)
        : frame_id(frame_id), fearture_points(features)
    {
    }

    feature_and_id_pair fearture_points;
    // std::map<uint64_t, Eigen::Vector2i> fearture_points;

    uint64_t frame_id;

private:
};

/**
 * @brief 特徴点抽出部分の開発方針について
 * @details
 * 1. はじめに思いつく通りに実装してみる
 * 2. 一旦抽象化を行い再実装する
 * 
 */

class dense_feature_extructor
{
public:
    dense_feature_extructor();

    /**
     * @brief 特徴点の検出と追跡を両方行う
     * @details
     * 処理内容としては、特徴点の検出、トラッキング。処理フローは以下の通り。
     * 1. 特徴点の初期化
     * 2. 特徴点のトラッキング＋特徴点の追加
     *  - 特徴点追加は、トラッキングしている特徴点密度を維持する処理
     *  - 常に、グリッドで分けた領域にある程度以上の曲率頂点を保持するようにする
     * 
     * @param input_colored 
     */
    void detect_and_track(const cv::Mat &input_color);
    void detect_and_track_proto(const cv::Mat &input_color); // 試作バージョン

    // 曲率画像を生成する
    cv::Mat get_curavture(const cv::Mat &input_mono);

    // とにかくLocal maxを探索する
    cv::Point2i track_local_max(const cv::Mat &img_mono, const cv::Point2i &initial_point);
    // 探索開始点をアンカーとしてLocal maxを探索する
    cv::Point2i track_local_max_with_regularization(const cv::Mat &img_mono, const cv::Point2i &initial_point);

    cv::Point2i get_neighbor_max(const cv::Mat &img_mono, const cv::Point2i &input_point);
    cv::Point2i get_neighbor_max_with_regularization(const cv::Mat &img_mono,
                                                     const cv::Point2i &input_point,
                                                     const double lambda_coeff,
                                                     const double sigma_coeff,
                                                     const cv::Point2f &estimated_point);

    // 大域的な画像のフローを求める。
    cv::Mat get_dominant_flow(const cv::Mat &img_mono);

    const std::vector<cv::Point2f> &get_feature_points() const;

private:
    // 特徴点のトラッキング結果が入れられる
    std::vector<dense_feature> features;
    // // トラッキング中のインデックスを保持しておく
    // std::vector<uint64_t> tracking_indices;

    std::vector<frame_dense_feature> feature_with_frame;

    std::vector<cv::Point2f> feature_points;
    bool is_initialize;

    cv::Mat prev_descriptor;
    std::vector<cv::KeyPoint> prev_keypoints;
    bool is_initialize_dominant_flow;
    cv::Mat prev_img;

    double get_regularization_term(const double lambda_coeff,
                                   const double sigma_coeff,
                                   const cv::Point2f &input_point,
                                   const cv::Point2f &estimated_point);

    bool warp_point(
        const cv::Point2f &input,
        const cv::Mat &affine_mat,
        const cv::Size &image_size,
        cv::Point2f &output);

    std::vector<cv::Point2f> warp_points(
        const std::vector<cv::Point2f> &inputs,
        cv::Mat &affine_mat,
        cv::Size image_size);

    std::vector<dense_feature> initialize_fearure(
        const cv::Mat &img_curvature,
        const std::vector<cv::Point2f> &pre_points,
        const int32_t num_points = 10000);

    /**
     * @brief フレーム中の特徴点を初期化する
     * 
     * @param img_curvature 
     * @param prev_features 
     * @param num_points 
     * @return frame_dense_feature 
     */
    frame_dense_feature initialize_features(
        const cv::Mat &img_curvature,
        const frame_dense_feature &prev_features,
        const int32_t num_points = 10000);

    std::vector<dense_feature> remove_duplicated_feature(std::vector<dense_feature> &features, const cv::Size &img_size);
};
