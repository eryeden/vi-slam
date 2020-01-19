#pragma once

#include <iostream>
#include <vector>
#include <map>
#include <random>
#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>

#include "feature_in_frame.hpp"

namespace initializer
{

namespace utils
{
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

} // namespace utils

class initializer
{
public:
private:
};

} // namespace initializer