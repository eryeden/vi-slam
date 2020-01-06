#pragma once

#include <iostream>
#include <vector>
#include <map>
#include <random>
#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>

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

    void run_extruction(const std::string &path_to_log_dir);
    void run_extruction_cam(const std::string &path_to_log_dir, double scale);

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

    cv::Mat get_curavture(const cv::Mat &input_color);

    cv::Point2i get_neighbor_max(const cv::Mat &img_mono, const cv::Point2i &input_point);
    cv::Point2i get_neighbor_max_with_regularization(const cv::Mat &img_mono,
                                                     const cv::Point2i &input_point,
                                                     const double lambda_coeff,
                                                     const double sigma_coeff,
                                                     const cv::Point2f &estimated_point);
    cv::Point2i track_local_max(const cv::Mat &img_mono, const cv::Point2i &initial_point);

    cv::Mat get_dominant_flow(const cv::Mat &img_color);

private:
    // 特徴点のトラッキング結果が入れられる
    std::map<uint64_t, dense_feature> features;

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
};
