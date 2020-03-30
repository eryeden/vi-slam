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

/**
 * @brief LandmarkをFrameの観測情報に従って更新する
 * @param landmark_database
 * @param input_frame
 */
void update_landmark_database(LogPlayer_vio_dataset::landmark_database_t &landmark_database,
                              const vislam::data::frame &input_frame) {
  // ランドマークデータベースに今回のFrameで検出したものを登録する
  for (const auto &[landmark_id, landmark_position] : input_frame.observingFeaturePointInDevice) {
    if (landmark_database.count(landmark_id) != 0) { // databaseに登録済みの場合
      landmark_database[landmark_id].isTracking = true;
      landmark_database[landmark_id].observedFrameId.emplace(input_frame.id);
    } else {
      // databaseに登録なしの場合
      landmark_database[landmark_id] = vislam::data::landmark(landmark_id,
                                                              {input_frame.id},
                                                              {0, 0, 0},
                                                              false,
                                                              true,
                                                              false);
    }
  }
}

bool try_initialize_system(LogPlayer_vio_dataset::frame_database_t &frame_database,
                           LogPlayer_vio_dataset::landmark_database_t &landmark_database,
                           uint64_t latest_frame_index,
                           double match_rate_threshold) {
  if (frame_database.size() < 2) {
    return false;
  }
  const auto &reference_frame = frame_database[1];
  const auto &current_frame = frame_database[latest_frame_index];
  std::vector<vislam::data::landmark> localized_landmarks;
  vislam::Vec3_t initialized_position_current_frame;
  vislam::Quat_t initialized_attitude_current_frame;
  double match_rate = initializer::utils::initialize_feature_points(reference_frame, current_frame,
                                                                    localized_landmarks,
                                                                    initialized_position_current_frame,
                                                                    initialized_attitude_current_frame);
  std::cout << "Match rate: " << match_rate << std::endl;

  if (match_rate > match_rate_threshold) {
    std::cout << "Initialized. Match rate : " << match_rate << std::endl;

    /**
     * @brief 特徴点の観測情報を更新する
     * @details
     * ここではLandmark databaseの位置情報、初期化情報などを更新している。
     * なので、次からはLandmarkDatabaseを参照することで初期化結果を利用できる
     */
    for (const auto &lm :  localized_landmarks) {
      auto &related_landmark = landmark_database[lm.id];
      related_landmark.isTracking = lm.isTracking;
      related_landmark.isOutlier = lm.isOutlier;
      related_landmark.isInitialized = lm.isInitialized;
      related_landmark.positionInWorld = lm.positionInWorld;
      related_landmark.id = lm.id;
    }

    /**
     * @brief 今回の初期化フレームの位置を設定する
     */
    auto &ref_ref_frame = frame_database[1];
    ref_ref_frame.cameraPosition = vislam::Vec3_t(0, 0, 0);
    ref_ref_frame.cameraAttitude = vislam::Quat_t(vislam::Mat33_t::Identity());
    auto &ref_current_frame = frame_database[latest_frame_index];
    ref_current_frame.cameraPosition = initialized_position_current_frame;
    ref_current_frame.cameraAttitude = initialized_attitude_current_frame;

    /**
     * いままでのFramePoseを初期化する。初期化できたLandmark位置からPNPでPoseを求める
     */
    for (size_t initializing_frame_index = 2; initializing_frame_index < latest_frame_index;
         initializing_frame_index++) {
      auto &tmp_frame = frame_database[initializing_frame_index];
      vislam::Vec3_t tmp_translation;
      vislam::Quat_t tmp_rotation;
      initializer::utils::estimate_frame_pose_pnp(tmp_frame, landmark_database, tmp_translation, tmp_rotation);
      tmp_frame.cameraPosition = tmp_translation;
      tmp_frame.cameraAttitude = tmp_rotation;
    }

    /**
     * @brief ここまでのFrameと初期化できたLandmarkでBAを実施する
     */
    std::unordered_map<uint64_t, vislam::data::frame> ba_database_frame;
    ba_database_frame[1] = frame_database[1];
    ba_database_frame[latest_frame_index] = frame_database[latest_frame_index];
    std::vector<vislam::ba::ba_observation> selected_observation_database;
    std::vector<uint64_t> selected_landmark_id;

    //! Do the BA
    std::unordered_map<uint64_t, vislam::data::frame> opt_database_frame;
    std::unordered_map<uint64_t, vislam::data::landmark> opt_database_landmark;
    vislam::ba::ba_pre::do_the_ba(
        ba_database_frame,
        landmark_database,
        opt_database_frame,
        opt_database_landmark);
    for (const auto&[id, f]:opt_database_frame) {
      frame_database[id] = f;
    }
    for (const auto&[id, l]:opt_database_landmark) {
      landmark_database[id] = l;
    }
    return true;
  }

  return false;
}

int main() {

  std::string path_to_log_dir = "/home/ery/Downloads/V1_01_easy/mav0/cam0";
//  std::string path_to_log_dir = "/e/subspace/tmp/tmp/V1_01_easy/mav0/cam0";
  load_and_detect_frame_euroc loader(path_to_log_dir, 0.1, 0.1);

  LogPlayer_vio_dataset::frame_database_t frame_database;
  LogPlayer_vio_dataset::landmark_database_t landmark_database;

  bool is_initialized = false;

  cv::viz::Viz3d myWindow("Coordinate Frame");
  myWindow.showWidget("Coordinate Widget", cv::viz::WCoordinateSystem());

  /**
   * @brief Dummyで一回回す
   */
  loader.get_frame();

  for (size_t frame_index = 0; frame_index < loader.get_frame_number(); frame_index++) {
    /**
     * @brief Landmark検出、トラッキング、データベースのアップデート
     */
    auto frame = loader.get_frame();  // 特徴点の検出追跡、読み取りなど
    cv::Mat current_frame = loader.get_current_image(); // 今回の画像
    frame_database[frame.id] = frame; // 観測フレームのデータベースに登録する
    update_landmark_database(landmark_database, frame); // ランドマークデータベースに今回のFrameで検出したものを登録する

    /**
     * @brief BAプロセス
     */
    if (!is_initialized) {
      // 最初のフレームを初期化していない場合
      if (try_initialize_system(frame_database, landmark_database, frame_index, 0.5)) {
        is_initialized = true;
      }
    } else {
      // 初期化が完了している場合
      /**
       * @brief 1. current frameの位置、姿勢を初期化
       */
      vislam::Vec3_t position_current_frame;
      vislam::Quat_t attitude_current_frame;
      initializer::utils::estimate_frame_pose_pnp(frame,
                                                  landmark_database,
                                                  position_current_frame,
                                                  attitude_current_frame);
      //! p3pで計算した現在フレームの位置をDataBaseに登録する
      frame_database[frame_index].cameraPosition = position_current_frame;
      frame_database[frame_index].cameraAttitude = attitude_current_frame;

      /**
       * @brief 2. Landmark位置の初期化を実施する
       * @details
       * 基本、初期化（２視点から観測されている特徴点位置を三角測量で求める）は次の条件の特徴点に対して実施する。
       * - 初期化されていない
       * - 2 Frame以上観測されている
       * - 観測されているフレームの最大視差が、ある値以上になっている
       * - 今回のフレームで観測されている
       */
      auto initialized_landmark =
          initializer::utils::extract_and_triangulate_initializable_landmark(10.0 * M_PI / 180.0,
                                                                             frame_index,
                                                                             frame_database,
                                                                             landmark_database);
      for (const auto &[landmark_id, landmark_data] : initialized_landmark) {
        landmark_database[landmark_id] = landmark_data; //! データベースに初期化済みLandmarkを登録
      }

      /**
       * @brief BAを実施する
       */
      if (frame_index % 5 == 0) {
        std::unordered_map<uint64_t, vislam::data::frame> ba_database_frame;
        int32_t ba_window_size = 3; // このフレーム数を使ってLocal baを実施する
        int32_t ba_window_skip = 3;
        for (int32_t ba_rollback_index = 0; ba_rollback_index < ba_window_size; ba_rollback_index += ba_window_skip) {
          ba_database_frame[frame_index - ba_rollback_index] = frame_database[frame_index - ba_rollback_index];
        }
        ba_database_frame[1] = frame_database[1];
        std::vector<vislam::ba::ba_observation> selected_observation_database;
        std::vector<uint64_t> selected_landmark_id;
        std::unordered_map<uint64_t, vislam::data::frame> opt_database_frame;
        std::unordered_map<uint64_t, vislam::data::landmark> opt_database_landmark;
        vislam::ba::ba_pre::do_the_ba(
            ba_database_frame,
            landmark_database,
            opt_database_frame,
            opt_database_landmark);
        for (const auto&[id, f]:opt_database_frame) {
          frame_database[id] = f;
        }
        for (const auto&[id, l]:opt_database_landmark) {
          landmark_database[id] = l;
        }
      }
    }


    /**
     * @brief カメラ位置、Landmark位置を描画
     */
    // 特徴点位置を描画
    std::vector<cv::Point3d> pointCloud;
    for (auto &[landmark_id, localized_landmark] : landmark_database) {
      pointCloud.emplace_back(
          cv::Point3d(localized_landmark.positionInWorld[0],
                      localized_landmark.positionInWorld[1],
                      localized_landmark.positionInWorld[2]));
    }
    cv::viz::WCloud cloud(pointCloud);
    myWindow.showWidget("CLOUD", cloud);

    for (const auto&[frame_id, frame] : frame_database) {
      cv::Matx33d cv_intrinsic_matrix;
      cv::eigen2cv(loader.get_camera_parameter().get_intrinsic_matrix(), cv_intrinsic_matrix);
      cv::viz::WCameraPosition tmp_wcamera(cv_intrinsic_matrix, 1.0, cv::viz::Color::magenta());
      cv::Mat tmp_camera_attitude;
      cv::eigen2cv(frame.cameraAttitude.toRotationMatrix(), tmp_camera_attitude);
      cv::Affine3d tmp_cam_pose(tmp_camera_attitude,
                                cv::Vec3f(frame.cameraPosition[0], frame.cameraPosition[1], frame.cameraPosition[2]));
      myWindow.showWidget(std::to_string(frame_id), tmp_wcamera, tmp_cam_pose);
    }


    /**
     * @brief 特徴点の描画
     */
    for (const auto&[id, p]: frame.observingFeaturePointInDevice) {
      cv::circle(current_frame, cv::Point(p(0), p(1)), 1, cv::Scalar(0, 255, 255), 1);
    }
    cv::imshow("current", current_frame);

    myWindow.spinOnce(1);
    cv::waitKey(1);
  }
}












