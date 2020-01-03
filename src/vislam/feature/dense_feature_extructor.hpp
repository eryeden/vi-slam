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

    cv::Mat get_curavture(const cv::Mat &input_color);

    cv::Point2i get_neighbor_max(const cv::Mat &img_mono, const cv::Point2i &input_point);

    cv::Point2i track_local_max(const cv::Mat &img_mono, const cv::Point2i &initial_point);

    void get_dominant_flow(const cv::Mat &img_mono);

private:
    std::vector<cv::Point2i> feature_points;
    bool is_initialize;
};