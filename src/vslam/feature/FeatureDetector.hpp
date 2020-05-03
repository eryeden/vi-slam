//
// Created by ery on 2020/05/03.
//

#pragma once

#include <opencv2/opencv.hpp>

#include "Frame.hpp"

/**
 * @brief Feature detection関係をここに入れておく。
 * @details
 * とりあえず、Shi-tomoasi cornerを使ったものを実装しておく。
 * SIFTは特許が切れたらしく利用したいが、あまり早くなく、高速化版のSURFは特許が取られているか、切れているか、
 * 不明。この部分は注力したい領域ではないので、とりあえずShi-tomasiで実装して、問題になるようなら
 * ほかの実装も検討する。
 * VITAMIN-Eは、Trackingの部分に実装する。
 */

namespace vslam::feature {

data::Frame DetectShiTomasiCorners(const data::FrameSharedPtr& previous_frame,
                                   const cv::Mat& undistorted_current_image,
                                   int32_t division_number_row,
                                   int32_t division_number_col,
                                   int32_t max_feature_number);

}