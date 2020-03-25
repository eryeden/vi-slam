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

int main() {

  /**
   * @brief Load simulation generated data
   */
  //{@
  LogPlayer_vio_dataset::frame_database_t input_frame_database;
  LogPlayer_vio_dataset::landmark_database_t input_landmark_database;
  //! vio_data_simulationで生成したデータ群のトップディレクトリを書いておく
  std::string path_to_log = "/home/ery/Works/Devel/tmp/vio_data_simulation/bin";
//  std::string path_to_log = "/home/anudev/devel/vio_data_simulation/bin/";

  //! LogPlayerの生成
  LogPlayer_vio_dataset logplayer(path_to_log);

  //! data baseの構築
  logplayer.generate_database_form_file(input_frame_database, input_landmark_database);

  //! カメラパラメータの登録
  vislam::data::camera camera_pram(0,
                                   640, 480,
                                   30,
                                   458.654, 457.296, 367.215, 248.375, //! eigenのmatからだとうまいこといかなかった
                                   0, 0, 0, 0, 0);
  for (auto &[frame_id, frame]: input_frame_database) {
    frame.cameraParameter = camera_pram;
  }
  //@}

  /**
   * @brief BA用のノイズ印加データをつくる
   */
  LogPlayer_vio_dataset::frame_database_t noised_frame_database;
  LogPlayer_vio_dataset::landmark_database_t noised_landmark_database;

  // 乱数生成器
  std::random_device seed_gen;
  std::default_random_engine engine(seed_gen());
  // 平均0.0、標準偏差1.0で分布させる
  std::normal_distribution<> dist(0.0, 1.0);
  std::uniform_real_distribution<> uniform_dist(-1, 1);
  std::normal_distribution<> rotation_angle_dist(0.0, 10.0 * M_PI / 180.0);

  int32_t frame_skip_size = 10;
  for (const auto&[frame_id, frame] : input_frame_database) {
    if ((frame_id % frame_skip_size == 0) && (frame_id < 300)) {
      noised_frame_database[frame_id] = frame;

      //! add random noise
      noised_frame_database[frame_id].cameraPosition(0) += dist(engine);
      noised_frame_database[frame_id].cameraPosition(1) += dist(engine);
      noised_frame_database[frame_id].cameraPosition(2) += dist(engine);

      //! add random rotation
      vislam::Vec3_t random_direction_vector{uniform_dist(engine), uniform_dist(engine), uniform_dist(engine)};
      random_direction_vector.normalize();
      Eigen::AngleAxisd angle_axisd(rotation_angle_dist(engine), random_direction_vector);
      vislam::Mat33_t random_rotation_matrix = angle_axisd.toRotationMatrix();
      vislam::Mat33_t noised_orientation = random_rotation_matrix * frame.cameraAttitude.toRotationMatrix();
      noised_frame_database[frame_id].cameraAttitude = vislam::Quat_t(noised_orientation);

      for (const auto &[landmark_id, landmark_position] : frame.observingFeaturePointInDevice) {
        if (noised_landmark_database.count(landmark_id) != 0) { // databaseに登録済みの場合
          noised_landmark_database[landmark_id].isTracking = true;
          noised_landmark_database[landmark_id].observedFrameId.emplace(frame_id);
        } else { // databaseに登録なしの場合
          vislam::Vec3_t gt_position = input_landmark_database[landmark_id].positionInWorld;
          //! add random noise
          gt_position(0) += dist(engine);
          gt_position(1) += dist(engine);
          gt_position(2) += dist(engine);

          noised_landmark_database[landmark_id] = vislam::data::landmark(landmark_id,
                                                                         {frame_id},
                                                                         gt_position,
                                                                         false,
                                                                         true,
                                                                         true
          );
        }
      }
    }
  }




  /**
   * @brief 描画関係
   */
  //@{
  cv::Matx33d K;
  K << 458.654, 0.0000000000000000e+00, 367.215,
      0.0000000000000000e+00, 457.296, 248.375,
      0.0000000000000000e+00, 0.0000000000000000e+00, 1.0000000000000000e+00;
  cv::viz::Viz3d myWindow("Coordinate Frame");
  myWindow.showWidget("Coordinate Widget", cv::viz::WCoordinateSystem());
  //@}


  /**
 * @brif 描画関係処理
 */

  // 特徴点位置を描画
  std::vector<cv::Point3d> pointCloud;
  for (auto &[landmark_id, localized_landmark] : noised_landmark_database) {
    pointCloud.emplace_back(
        cv::Point3d(localized_landmark.positionInWorld[0],
                    localized_landmark.positionInWorld[1],
                    localized_landmark.positionInWorld[2]));
  }
  cv::viz::WCloud cloud(pointCloud);
  myWindow.showWidget("CLOUD", cloud);

  for (const auto&[frame_id, frame] : noised_frame_database) {
    cv::viz::WCameraPosition tmp_wcamera(K, 1.0, cv::viz::Color::magenta());
    cv::Mat tmp_camera_attitude;
    cv::eigen2cv(frame.cameraAttitude.toRotationMatrix(), tmp_camera_attitude);
    cv::Affine3d tmp_cam_pose(tmp_camera_attitude,
                              cv::Vec3f(frame.cameraPosition[0], frame.cameraPosition[1], frame.cameraPosition[2]));
    myWindow.showWidget(std::to_string(frame_id), tmp_wcamera, tmp_cam_pose);

    auto text_3d = cv::viz::WText3D(std::to_string(frame_id),    // 表示するテキスト
                                    cv::Point3d(frame.cameraPosition[0],
                                                frame.cameraPosition[1],
                                                frame.cameraPosition[2]),    // 表示位置
                                    0.4,    // テキストのサイズ
                                    true,    // テキストが常に見えるようにするか？
                                    cv::viz::Color::black());
    myWindow.showWidget(std::to_string(frame_id) + "txt", text_3d);
  }

  for (const auto&[frame_id, frame_tmp] : noised_frame_database) {
    auto &frame = input_frame_database[frame_id];
    cv::viz::WCameraPosition tmp_wcamera(K, 1.0, cv::viz::Color::yellow());
    cv::Mat tmp_camera_attitude;
    cv::eigen2cv(frame.cameraAttitude.toRotationMatrix(), tmp_camera_attitude);
    cv::Affine3d tmp_cam_pose(tmp_camera_attitude,
                              cv::Vec3f(frame.cameraPosition[0], frame.cameraPosition[1], frame.cameraPosition[2]));
    myWindow.showWidget(std::to_string(frame_id) + "gt", tmp_wcamera, tmp_cam_pose);

    auto text_3d = cv::viz::WText3D(std::to_string(frame_id) + "gt",    // 表示するテキスト
                                    cv::Point3d(frame.cameraPosition[0],
                                                frame.cameraPosition[1],
                                                frame.cameraPosition[2]),    // 表示位置
                                    0.4,    // テキストのサイズ
                                    true,    // テキストが常に見えるようにするか？
                                    cv::viz::Color::black());
    myWindow.showWidget(std::to_string(frame_id) + "gttxt", text_3d);
  }

  myWindow.spin();


  //! Do the BA
  std::unordered_map<uint64_t, vislam::data::frame> opt_database_frame;
  std::unordered_map<uint64_t, vislam::data::landmark> opt_database_landmark;
  vislam::ba::ba_pre::do_the_ba(
      noised_frame_database,
      noised_landmark_database,
      opt_database_frame,
      opt_database_landmark);

  return 0;
}


