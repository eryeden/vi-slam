//
// Created by ery on 2020/05/14.
//

#pragma once

#include <opencv2/opencv.hpp>

#include "FeatureDetectorBase.hpp"
#include "Frame.hpp"
#include "NonMaximumSuppression/NonMaximumSuppression.hpp"

namespace vslam::feature {

enum class FeatureDetectorType { FAST, ORB, AGAST, GFTT };

class FeatureDetectorANMS : public FeatureDetectorBase {
 public:
  class Parameter {
   public:
    Parameter();

    /**
     * @brief 最終的に出力するFeatureの設定
     */
    int32_t max_feature_number_;
    double min_feature_distance_;

    /**
     * @brief Internal detector
     */
    int32_t detection_feature_number_;
    double detection_min_feature_distance_;
    double detection_quality_level_;
    int32_t detection_block_size_;
    bool detection_use_harris_corner_;
    double detection_k_;
    int32_t detection_ignore_edge_;

    /**
     * @brief ANMS
     */
    double non_max_suppression_tolerance_;
  };

 public:
  explicit FeatureDetectorANMS(const Parameter parameter);

  void UpdateDetection(FeaturePositionDatabase& feature_position,
                       FeatureAgeDatabase& feature_age,
                       const cv::Mat& current_image,
                       const cv::Mat& mask_image = cv::Mat()) override;

 private:
  void Detect(FeaturePositionDatabase& feature_position,
              FeatureAgeDatabase& feature_age,
              const cv::Mat& current_image,
              int32_t max_feature_number,
              double min_feature_distance,
              database_index_t& max_feature_index,
              const cv::Mat& mask_image = cv::Mat()) const;

  database_index_t max_feature_index_;

  Parameter parameter_;

  std::unique_ptr<NonMaximumSuppression> non_max_suppression_;

  // Actual feature detector implementation.
  cv::Ptr<cv::Feature2D> feature_detector_;
};

}  // namespace vslam::feature