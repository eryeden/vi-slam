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

  /**
   * @brief Load simulation generated data
   */
  //{@
  LogPlayer_vio_dataset::frame_database_t frame_database;
  LogPlayer_vio_dataset::landmark_database_t landmark_database;
  //! vio_data_simulationで生成したデータ群のトップディレクトリを書いておく
  std::string path_to_log = "/home/ery/Works/Devel/tmp/vio_data_simulation/bin";
//  std::string path_to_log = "/home/anudev/devel/vio_data_simulation/bin/";

  //! LogPlayerの生成
  LogPlayer_vio_dataset logplayer(path_to_log);

  //! data baseの構築
  logplayer.generate_database_form_file(frame_database, landmark_database);

  //! カメラパラメータの登録
  vislam::data::camera camera_pram(0,
                                   640, 480,
                                   30,
                                   458.654, 457.296, 367.215, 248.375, //! eigenのmatからだとうまいこといかなかった
                                   0, 0, 0, 0, 0);
  for (auto &[frame_id, frame]: frame_database) {
    frame.cameraParameter = camera_pram;
  }
  //@}

  /**
   * @brief BA process
   */
  //@{

  //! @brief 観測したフレームの保存場所
  std::unordered_map<uint64_t, vislam::data::frame> database_frame;
  //! @brief 観測した特徴点（Feature point, landmark）の保存場所
  std::unordered_map<uint64_t, vislam::data::landmark> database_landmark;
  //! 初期化成功したか？
  bool is_initialized = false;
  //! 初期化判定を行うMatch rateのしきい値
  double match_rate_threshold = 0.5;


  //! show detected landmarks
  cv::Mat out(480, 640, CV_8UC3);
  out = 0;

  for (size_t frame_id = 0; frame_id < frame_database.size(); frame_id++) {
    const auto &input_frame = frame_database[frame_id];

    // 観測したフレームを登録する
    database_frame[frame_id] = input_frame;

    // ランドマーク位置の更新
    for (const auto &[landmark_id, landmark_position] : input_frame.observingFeaturePointInDevice) {
      if (database_landmark.count(landmark_id) != 0) { // databaseに登録済みの場合
        database_landmark[landmark_id].isTracking = true;
        /**
         * @brief std::setへの要素追加が発生している。ここでの処理コストが大きい場合はstd::vectorへの置き換えが必要かもしれない。
         * 結局、カメラIDは増加しかしないため、std::vectorで追加しつつ、std::set_intersectionを行っても同じなのかもしれない。
         */
        database_landmark[landmark_id].observedFrameId.emplace(input_frame.id);
      } else { // databaseに登録なしの場合
        database_landmark[landmark_id] = vislam::data::landmark(landmark_id,
                                                                {input_frame.id},
                                                                {0, 0, 0},
                                                                false,
                                                                true,
                                                                false
        );
      }
    }

    if (!is_initialized) {
      if (database_frame.size() > 2) {

        /**
         * @brief 特徴点位置の初期化を試みる
         */
        const auto &reference_frame = database_frame[0]; // index:0のframeは何も入っていなかったの注意
        const auto &current_frame = database_frame[frame_id];
        std::vector<vislam::data::landmark> localized_landmarks;

        vislam::Vec3_t initialized_position_current_frame;
        vislam::Quat_t initialized_attitude_current_frame;

        double match_rate = initializer::utils::initialize_feature_points(reference_frame, current_frame,
                                                                          localized_landmarks,
                                                                          initialized_position_current_frame,
                                                                          initialized_attitude_current_frame);

        std::cout << "Match rate: " << match_rate << std::endl;

        if (match_rate > match_rate_threshold) {

          /**
           * @brief 特徴点位置の初期化を完了して特徴点データベースの更新を行う
           */
          is_initialized = true;
          std::cout << "Initialized. Match rate : " << match_rate << std::endl;

          /**
           * @brief 特徴点の観測情報を更新する
           * @details
           * ここではLandmark databaseの位置情報、初期化情報などを更新している。
           * なので、次からはLandmarkDatabaseを参照することで初期化結果を利用できる
           */
          for (const auto &lm :  localized_landmarks) {
            auto &related_landmark = database_landmark[lm.id];
            related_landmark.isTracking = lm.isTracking;
            related_landmark.isOutlier = lm.isOutlier;
            related_landmark.isInitialized = lm.isInitialized;
            related_landmark.positionInWorld = lm.positionInWorld;
            related_landmark.id = lm.id;
          }

          /**
           * @brief 今回の初期化フレームの位置を設定する
           */
          auto &ref_ref_frame = database_frame[1];
          ref_ref_frame.cameraPosition = vislam::Vec3_t(0, 0, 0);
          ref_ref_frame.cameraAttitude = vislam::Quat_t(vislam::Mat33_t::Identity());
          auto &ref_current_frame = database_frame[i];
          ref_current_frame.cameraPosition = initialized_position_current_frame;
          ref_current_frame.cameraAttitude = initialized_attitude_current_frame;

          /**
           * いままでのFramePoseを初期化する。初期化できたLandmark位置からPNPでPoseを求める
           */
          for (size_t initializing_frame_index = 2;
               initializing_frame_index < i; initializing_frame_index++) {
            auto &tmp_frame = database_frame[initializing_frame_index];
            vislam::Vec3_t tmp_translation;
            vislam::Quat_t tmp_rotation;
            initializer::utils::estimate_frame_pose_pnp(tmp_frame, database_landmark,
                                                        tmp_translation, tmp_rotation);
            tmp_frame.cameraPosition = tmp_translation;
            tmp_frame.cameraAttitude = tmp_rotation;
          }

          /**
           * @brief ここまでのFrameと初期化できたLandmarkでBAを実施する
           */
          std::unordered_map<uint64_t, vislam::data::frame> ba_database_frame;
          ba_database_frame[1] = database_frame[1];
          for (size_t window = 0; window <= 3; window++) {
            ba_database_frame[frame_id - window] = database_frame[frame_id - window];
          }
//          ba_database_frame[i - 2] = database_frame[i - 2];
//          ba_database_frame[i - 1] = database_frame[i - 1];
//          ba_database_frame[i] = database_frame[i];
          std::vector<vislam::ba::ba_observation> selected_observation_database;
          std::vector<uint64_t> selected_landmark_id;

          //! Do the BA
          std::unordered_map<uint64_t, vislam::data::frame> opt_database_frame;
          std::unordered_map<uint64_t, vislam::data::landmark> opt_database_landmark;
          vislam::ba::ba_pre::do_the_ba(
              ba_database_frame,
//                            database_frame,
              database_landmark,
              opt_database_frame,
              opt_database_landmark);

          for (const auto&[id, f]:opt_database_frame) {
            database_frame[id] = f;
          }
          for (const auto&[id, l]:opt_database_landmark) {
            database_landmark[id] = l;
          }


          /**
           * @brif 描画関係処理
           */

          // 特徴点位置を描画
          std::vector<cv::Point3d> pointCloud;
          for (auto &localized_landmark : localized_landmarks) {
            if (!localized_landmark.isOutlier) {
              pointCloud.emplace_back(
                  cv::Point3d(localized_landmark.positionInWorld[0],
                              localized_landmark.positionInWorld[1],
                              localized_landmark.positionInWorld[2]));
            }
          }
          // 点群の描画
          cv::viz::WCloud cloud(pointCloud);
          myWindow.showWidget("CLOUD", cloud);

          // 基準カメラ位置の描画
          cv::Affine3d initial_cam_pose(cv::Mat::eye(3, 3, CV_64FC1), cv::Vec3f(0, 0, 0));
          myWindow.showWidget("1", wcamera, initial_cam_pose);
          // 初期化カメラ位置の描画
          cv::Mat current_camera_attitude;
          cv::eigen2cv(ref_current_frame.cameraAttitude.toRotationMatrix(), current_camera_attitude);
          cv::Affine3d current_cam_pose(current_camera_attitude,
                                        cv::Vec3f(ref_current_frame.cameraPosition[0],
                                                  ref_current_frame.cameraPosition[1],
                                                  ref_current_frame.cameraPosition[2]));
          myWindow.showWidget("2", wcamera_cand1, current_cam_pose);
          // 初期化後、前フレームカメラ位置の描画
          for (size_t initializing_frame_index = 2;
               initializing_frame_index < system_init_frame_id; initializing_frame_index++) {
            const auto &tmp_frame = database_frame[initializing_frame_index];
            cv::viz::WCameraPosition tmp_wcamera(K, 1.0, cv::viz::Color::magenta());
            cv::Mat tmp_camera_attitude;
            cv::eigen2cv(tmp_frame.cameraAttitude.toRotationMatrix(), tmp_camera_attitude);
            cv::Affine3d tmp_cam_pose(tmp_camera_attitude,
                                      cv::Vec3f(tmp_frame.cameraPosition[0], tmp_frame.cameraPosition[1],
                                                tmp_frame.cameraPosition[2]));
            myWindow.showWidget(std::to_string(initializing_frame_index), tmp_wcamera, tmp_cam_pose);
          }
        }
      }
    }

    out = 0;

    // draw points
    for (const auto &[landmark_id, p] : input_frame.observingFeaturePointInDevice) {
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


