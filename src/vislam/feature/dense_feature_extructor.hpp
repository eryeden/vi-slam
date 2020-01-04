#pragma once

#include <iostream>
#include <vector>
#include <random>
#include <opencv2/opencv.hpp>
#include <omp.h>

#include "log_util.h"

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

    // cv::Point2f

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