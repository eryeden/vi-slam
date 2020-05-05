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

namespace utility {
std::vector<cv::Rect2f> GenerateGrid(const cv::Size& size,
                                     int32_t division_number_col,
                                     int32_t division_number_row);
}

class FeatureDetectorShiTomasi {
 public:
  using FeaturePositionDatabase =
      EigenAllocatedUnorderedMap<database_index_t, Vec2_t>;
  using FeatureAgeDatabase = std::unordered_map<database_index_t, uint32_t>;

  FeatureDetectorShiTomasi(int32_t division_number_row,
                           int32_t division_number_col,
                           int32_t max_feature_number,
                           double min_feature_distance);

  void UpdateDetection(FeaturePositionDatabase& feature_position,
                       FeatureAgeDatabase& feature_age,
                       const cv::Mat& current_image);

 private:
  void DetectShiTomasiCorners(FeaturePositionDatabase& feature_position,
                              FeatureAgeDatabase& feature_age,
                              const cv::Mat& current_image,
                              int32_t division_number_row,
                              int32_t division_number_col,
                              int32_t max_feature_number,
                              double min_feature_distance,
                              database_index_t& max_feature_index) const;

  database_index_t max_feature_index_;

  int32_t division_number_row_;
  int32_t division_number_col_;
  int32_t max_feature_number_;
  double min_feature_distance_;
};

}  // namespace vslam::feature