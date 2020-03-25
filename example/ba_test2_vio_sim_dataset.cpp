//
// Created by ery on 2020/03/22.
//

/**
 * @file sample2_vio_sim_dataset.cpp
 * @brief シミュレーションによって生成したVIOデータセットを使って、己のBAアルゴリズムが正しく動いているか検証する。
 * @details
 * おれはEUROCでの特徴点にOutlierが多すぎるので、Robust const functionのない俺の手法では安定して収束していなかったと思っている。
 * だから、正しさをある程度調整できるシミュレーション生成データで、My algorithmの性能、収束性を測る。
 */

#include <iostream>
#include <algorithm>

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

int main() {

  LogPlayer_vio_dataset::frame_database_t load_frame_database;
  LogPlayer_vio_dataset::landmark_database_t load_landmark_database;
  //! vio_data_simulationで生成したデータ群のトップディレクトリを書いておく
  std::string path_to_log = "/home/ery/Works/Devel/tmp/vio_data_simulation/bin";
//  std::string path_to_log = "/home/anudev/devel/vio_data_simulation/bin/";

  //! LogPlayerの生成
  LogPlayer_vio_dataset logplayer(path_to_log);

  //! data baseの構築
  logplayer.generate_database_form_file(load_frame_database, load_landmark_database);

  //! カメラパラメータの登録
  vislam::data::camera camera_pram(0,
                                   640, 480,
                                   30,
                                   458.654, 457.296, 367.215, 248.375, //! eigenのmatからだとうまいこといかなかった
                                   0, 0, 0, 0, 0);
  for (auto &[frame_id, frame]: load_frame_database) {
    frame.cameraParameter = camera_pram;
  }

  //! show detected landmarks
  cv::Mat out(480, 640, CV_8UC3);
  out = 0;

  for (size_t frame_id = 0; frame_id < load_frame_database.size(); frame_id++) {
    const auto &frame = load_frame_database[frame_id];
    out = 0;



    // draw points
    for (const auto &[landmark_id, p] : frame.observingFeaturePointInDevice) {
      cv::circle(out, cv::Point(p(0), p(1)), 1.0, cv::Scalar(255, 255, 0), 1);
      cv::putText(out,
                  std::to_string(landmark_id),
                  cv::Point(p(0), p(1)),
                  CV_FONT_HERSHEY_PLAIN,
                  1,
                  cv::Scalar(0, 0, 255),
                  1,
                  CV_AA);
    }
    cv::putText(out,
                "Frame : " + std::to_string(frame_id),
                cv::Point(0, 50),
                CV_FONT_HERSHEY_PLAIN,
                1.5,
                cv::Scalar(0, 255, 255),
                1,
                CV_AA);
    cv::imshow("Features", out);
    cv::waitKey(30);
  }

  return 0;
}


