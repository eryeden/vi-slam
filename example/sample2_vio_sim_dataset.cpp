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

  LogPlayer_vio_dataset::frame_database_t frame_database;
  LogPlayer_vio_dataset::landmark_database_t landmark_database;

  //! vio_data_simulationで生成したデータ群のトップディレクトリを書いておく
  std::string path_to_log = "/home/ery/Works/Devel/tmp/vio_data_simulation/bin";

  //! LogPlayerの生成
  LogPlayer_vio_dataset logplayer(path_to_log);

  //! data baseの構築
  logplayer.generate_database_form_file(frame_database, landmark_database);

  return 0;
}


