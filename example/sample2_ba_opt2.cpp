//
// Created by ery on 2020/03/30.
//

/**
 * @file sample2_ba_opt2.cpp
 * @brief V-SLAM処理全体を実装してみて実装するにあたっての問題を抽出してみる。
 * @details
 * ## このプログラムでやりたいこと
 * - V-SLAM全体を実装してみて、自分のVSLAM理解を助ける。
 * - 実装上の問題を抽出する
 *
 * ## 実装の方針
 * - シミュレーションデータとEUROCを両方共使える感じで実装する
 * - data::frameとしてLandmarkの観測ができれば、その後は同じ処理になるはずなので、data::frameを作れるような関数を用意する。
 *
 */

#include <iostream>
#include <algorithm>
#include <random>

#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/viz.hpp>

#include <Eigen/Core>
#include <Eigen/SparseCholesky>
#include <Eigen/SparseQR>

#include <opencv2/core/eigen.hpp>

#include "frame.hpp"
#include "landmark.hpp"
#include "camera.hpp"
#include "ba_pre.hpp"
#include "dense_feature_extructor.hpp"
#include "initializer.hpp"
#include "log_util.h"

/**
 * @brief 画像ロード、特徴点追跡、Frameとしての出力まで行うクラス
 * @details VSLAM結果を特徴点追跡に利用することはできない構造なので、必要になったら大幅な変更が必要かもしれない
*/
class frame_loader_base {
 public:
  frame_loader_base() = default;
  virtual ~frame_loader_base() = default;

  virtual vislam::data::frame get_frame() = 0;
  virtual const cv::Mat &get_current_image() const = 0;
  virtual vislam::data::camera get_camera_parameter() const = 0;
  virtual uint64_t get_frame_number() = 0;
};

/**
 * @brief
 */
class load_and_detect_frame_euroc : public frame_loader_base {
 public:
  load_and_detect_frame_euroc(const std::string &path_to_euroc_log_dir,
                              double feature_tracker_coefficient_lambda,
                              double feature_tracker_coefficient_sigma)
      : dfe(feature_tracker_coefficient_lambda, feature_tracker_coefficient_sigma),
        lp_mav(path_to_euroc_log_dir, 0.001),
        current_index(0) {
    cv::Mat dummy_image;
    double dummy_timestamp;
    lp_mav.get_frame_by_index(dummy_image, dummy_timestamp, 0);
    std::vector<double> distortion_coeffs_array = {-0.28340811, 0.07395907, 0.00019359, 1.76187114e-05};
    camera_param = vislam::data::camera(0,
                                        dummy_image.size().width,
                                        dummy_image.size().height,
                                        30,
                                        458.654,
                                        457.296,
                                        367.215,
                                        248.375, //! eigenのmatからだとうまいこといかなかった
                                        distortion_coeffs_array[0],
                                        distortion_coeffs_array[1],
                                        distortion_coeffs_array[2],
                                        distortion_coeffs_array[3],
                                        distortion_coeffs_array[4]);

    camera_intrinsic_matrix = (cv::Mat_<double>(3, 3) << 458.654, 0.0000000000000000e+00, 367.215,
        0.0000000000000000e+00, 457.296, 248.375,
        0.0000000000000000e+00, 0.0000000000000000e+00, 1.0000000000000000e+00);
    camera_distortion_coeffs = (cv::Mat_<float>(5, 1) << -0.28340811, 0.07395907, 0.00019359, 1.76187114e-05);
  }

  load_and_detect_frame_euroc()
      : load_and_detect_frame_euroc("/home/ery/Downloads/V1_01_easy/mav0/cam0", 0.1, 0.1) {}

  vislam::data::frame get_frame() override {

    /**
     * @brief 画像の読み込みと特徴点の検出、追跡
     */
    cv::Mat img, img_undistorted;
    double tstamp;
    lp_mav.get_frame_by_index(img, tstamp, current_index);
    cv::undistort(img, img_undistorted, camera_intrinsic_matrix, camera_distortion_coeffs);
    dfe.detect_and_track(img_undistorted, false);

    /**
     * @brief 描画用にモノクロ画像をRGB表現に変換する
     */
    cv::cvtColor(img_undistorted, current_image, CV_GRAY2BGR);

    /**
     * @brief 抽出した特徴点からdata::frameを生成する
     */
    vislam::data::frame frame_current;
    frame_current.id = current_index;

    /**
     * @brief 今回のフレームで抽出・トラックできた特徴点を挿入する。
     */
    auto feature_points_current = dfe.features[dfe.features.size() - 1];
    for (size_t index_current_feature = 0;
         index_current_feature < feature_points_current.featureIDs.size(); index_current_feature++) {

      /**
       * @brief Frame中に検出した特徴点を記録する
       */
      uint64_t current_feature_id = feature_points_current.featureIDs[index_current_feature];
      Eigen::Vector2i current_feature_position_in_device = feature_points_current.features[index_current_feature];
      frame_current.observingFeaturePointInDevice[current_feature_id] = {current_feature_position_in_device[0],
                                                                         current_feature_position_in_device[1]};
      frame_current.observingFeatureId.emplace(current_feature_id);
    }
    frame_current.cameraParameter = camera_param;

    current_index++;
    return frame_current;
  }
  const cv::Mat &get_current_image() const override {
    return current_image;
  }
  vislam::data::camera get_camera_parameter() const override {
    return camera_param;
  }

  uint64_t get_frame_number() override {
    return lp_mav.get_frame_size();
  }

 private:
  dense_feature::dense_feature_extructor dfe;
  LogPlayer_euroc_mav lp_mav;

  vislam::data::camera camera_param;
  cv::Mat camera_intrinsic_matrix;
  cv::Mat camera_distortion_coeffs;

  uint64_t current_index;
  cv::Mat current_image;

};

class load_and_detect_frame_vio_simulation : public frame_loader_base {
  load_and_detect_frame_vio_simulation() {

  }

  vislam::data::frame get_frame() override {

  }
  const cv::Mat &get_current_image() const override {
  }
  vislam::data::camera get_camera_parameter() const override {

  }
};

int main() {

  load_and_detect_frame_euroc loader("/home/ery/Downloads/V1_01_easy/mav0/cam0", 0.1, 0.1);
  for (size_t frame_index = 0; frame_index < loader.get_frame_number(); frame_index++) {
    auto frame = loader.get_frame();

    cv::Mat current_frame = loader.get_current_image();
    for (const auto&[id, p]: frame.observingFeaturePointInDevice) {
      cv::circle(current_frame, cv::Point(p(0), p(1)), 1, cv::Scalar(0, 255, 255), 1);
    }

    cv::imshow("current", current_frame);
    cv::waitKey(30);
  }
}












