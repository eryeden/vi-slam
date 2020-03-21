
#pragma once

#include <iostream>
#include <vector>
#include <map>
#include <random>

#include <opencv2/opencv.hpp>
#include <Eigen/Dense>

namespace dense_feature {

/**
 * @brief フレームふ含まれる特徴点を保存しておく。
 * @details
 * ## 特徴点の保存について
 * 特徴点にはIDが付与される。これが、別のstd::vectorに入れられていて、同インデックスのFeatureとIDが対応している。
 * 
 * 
 * 
 */
class feature_in_frame {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  feature_in_frame(const uint64_t frame_id,
                   const std::vector<uint64_t> &input_feature_ids,
                   const std::vector<Eigen::Vector2i> &input_features);
  feature_in_frame();

  // 画像中の特徴点
  std::vector<Eigen::Vector2i> features;
  std::vector<Eigen::Vector3d> featuresIn3d;

  std::vector<uint64_t> featureIDs;
  std::vector<int8_t> featureMasks;

  // フレーム固有のID
  uint64_t frameID;
  Eigen::Vector2i imageSizeWH;

  // このフレームの位置・姿勢
  Eigen::Matrix3d rotation;
  Eigen::Vector3d translation;

  // カメラ内部パラメータ
  Eigen::Matrix3d intrinsic;

 private:
};

}; // namespace dense_feature