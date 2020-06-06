//
// Created by ery on 2020/05/06.
//

#include <fmt/format.h>
#include <spdlog/spdlog.h>

#include "EurocKimeraDataProvider.hpp"
#include "EurocKimeraDataProviderRadialTangentialCameraModel.hpp"
#include "FeatureDetectorANMS.hpp"
#include "FeatureDetectorShiTomasiBucketing.hpp"
#include "FeatureTrackerLucasKanade.hpp"
#include "KimeraFrontend.hpp"
#include "Verification.hpp"
#include "ViewerViz.hpp"
#include "iSAM2Backend.hpp"

int main() {
  //  std::string path_to_euroc = "/home/ery/Downloads/V1_01_easy";
  //  std::string path_to_euroc = "/home/ery/Downloads/V2_01_easy";
  //  std::string path_to_euroc = "/home/ery/Downloads/MH_01_easy";
  //  vslam::dataprovider::EurocKimeraDataProviderRadialTangentialCameraModel
  //      euroc_kimera_data_provider(path_to_euroc);

  //  // EUROC
  std::string path_to_euroc =
      "/home/ery/subspace/docker_work/dataset/V1_01_easy";
  //    std::string path_to_euroc =
  //        "/home/ery/subspace/docker_work/dataset/V2_01_easy";
  std::string path_to_calibfile =
      "/home/ery/subspace/docker_work/dataset/basalt_calib/euroc_calib/"
      "calib_results/calibration.json";

  //  std::string path_to_euroc =
  //      "/home/ery/subspace/docker_work/dataset/dataset-corridor1_512_16";
  //  std::string path_to_calibfile =
  //      "/home/ery/subspace/docker_work/dataset/basalt_calib/tumvi_calib_data/"
  //      "results/calibration.json";

  vslam::dataprovider::EurocKimeraDataProvider euroc_kimera_data_provider(
      path_to_euroc, path_to_calibfile);

  //    std::string path_to_euroc =
  //        "/e/subspace/docker_work/dataset/fukuroi/camlog_2020-05-13-21-09-39/";
  //    std::string path_to_calibfile =
  //        "/e/subspace/docker_work/dataset/fukuroi/calib_result/calibration.json";
  //    std::string path_to_mask =
  //        "/e/subspace/docker_work/dataset/fukuroi/calib_result/vingette_0.png";
  //    vslam::dataprovider::EurocKimeraDataProvider euroc_kimera_data_provider(
  //        path_to_euroc, path_to_calibfile, path_to_mask);

  // Map database
  auto threadsafe_map_database_ptr =
      std::make_shared<vslam::data::ThreadsafeMapDatabase>();

  // Build detector
  auto shi_tomasi_detector_ptr =
      std::make_shared<vslam::feature::FeatureDetectorShiTomasiBucketing>(
          2, 2, 200, 5.0);

  auto anms_detector_ptr =
      std::make_shared<vslam::feature::FeatureDetectorANMS>(300, 15.0);

  // Build tracker
  auto kl_tracker_ptr =
      std::make_shared<vslam::feature::FeatureTrackerLucasKanade>(
          30, 0.1, 24, 4, 1);

  // Build verification
  auto verification_ptr =
      std::make_shared<vslam::verification::FeatureVerification5PointRANSAC>(
          1 * M_PI / 180.0, 150, 0.9);
  //  auto verification_ptr =
  //      std::make_shared<vslam::verification::FeatureVerification5PointRANSAC>(
  //          0.1 * M_PI / 180.0, 150, 0.9);

  /**
   * @note
   * ↓のように、実体のあるインスタンスのポインタからShared pointerをつくると、
   * Shared_ptrによるデストラクタコール？と実体の方のデストラクタコールがかぶるので
   * double free or corruptionが発生する可能性あり。
   */
  // Build frontend
  //  vslam::frontend::KimeraFrontend kimera_frontend(
  //      std::shared_ptr<vslam::data::ThreadsafeMapDatabase>(
  //          &threadsafe_map_database),
  //      std::shared_ptr<vslam::feature::FeatureDetectorShiTomasiBucketing>(
  //          &shi_tomasi_detector),
  //      std::shared_ptr<vslam::feature::FeatureTrackerLucasKanade>(&kl_tracker),
  //      100.0,
  //      100);

  vslam::frontend::KimeraFrontend kimera_frontend(
      threadsafe_map_database_ptr,
      //                                                  shi_tomasi_detector_ptr,
      anms_detector_ptr,
      kl_tracker_ptr,
      verification_ptr,
      10.0,
      250);

  vslam::backend::iSAM2Backend i_sam_2_backend(threadsafe_map_database_ptr);

  vslam::data::FrameSharedPtr prev_frame = nullptr;
  vslam::FeatureAgeDatabase prev_feature_age;
  vslam::FeaturePositionDatabase prev_feature_position;

  vslam::frontend::KimeraFrontendInputRadialTangentialCameraModel prev_input;
  bool is_initialized = false;
  bool is_update = true;
  bool is_reach_the_last = false;
  uint64_t counter = 0;

  // Generate viewer
  auto viewer = vslam::viewer::ViewerViz();
  // Feed drawing primitives
  viewer.PushPrimitive(vslam::viewer::CoordinateSystemPrimitive(
      "world_origin", vslam::Vec3_t(0, 0, 0), vslam::Quat_t::Identity()));
  vslam::EigenAllocatedVector<vslam::Vec3_t> estimated_trajectory;
  // Create a window and render primitives.
  viewer.LaunchViewer();

  while (!is_reach_the_last) {
    if (is_update) {
      auto input = euroc_kimera_data_provider.GetInput();
      if (input == std::nullopt) {
        is_reach_the_last = true;
        continue;
      }

      kimera_frontend.Feed(input.value());
      i_sam_2_backend.SpinOnce();
    }

    // visualize
    cv::Mat vis;
    //    kimera_frontend.last_input_.frame_.convertTo(vis, CV_8UC3);
    cv::cvtColor(kimera_frontend.last_input_.frame_, vis, CV_GRAY2BGR);

    auto latest_frame = threadsafe_map_database_ptr->GetFrame(
        threadsafe_map_database_ptr->latest_frame_id_);

    for (const auto& [id, pos] :
         latest_frame.lock()->observing_feature_point_in_device_) {
      int32_t landmark_age = latest_frame.lock()->feature_point_age_.at(id);
      if (landmark_age <= 1) {
        cv::circle(
            vis, cv::Point(pos[0], pos[1]), 2, cv::Scalar(0, 0, 255), 1, CV_AA);
      } else if (landmark_age == 2) {
        cv::circle(
            vis, cv::Point(pos[0], pos[1]), 3, cv::Scalar(255, 0, 0), 1, CV_AA);
      } else if (landmark_age > 2) {
        cv::circle(
            vis, cv::Point(pos[0], pos[1]), 3, cv::Scalar(0, 255, 0), 1, CV_AA);
      }

      if (id == 215) {
        cv::circle(
            vis, cv::Point(pos[0], pos[1]), 5, cv::Scalar(0, 255, 0), 1, CV_AA);
      }
      if (id == 227) {
        cv::circle(
            vis, cv::Point(pos[0], pos[1]), 5, cv::Scalar(0, 255, 0), 1, CV_AA);
      }
      if (id == 301) {
        cv::circle(
            vis, cv::Point(pos[0], pos[1]), 5, cv::Scalar(0, 255, 0), 1, CV_AA);
      }
    }

    // draw feature point number
    std::string str_feature_number = fmt::format(
        "Features : {}",
        latest_frame.lock()->observing_feature_point_in_device_.size());
    cv::putText(vis,
                str_feature_number,
                cv::Point(20, 20),
                CV_FONT_NORMAL,
                0.8,
                cv::Scalar(0, 0, 0),
                1,
                CV_AA);

    // Draw points
    vslam::EigenAllocatedVector<vslam::Vec3_t> pc_source;
    auto lms = threadsafe_map_database_ptr->GetAllLandmarks();
    for (const auto& [id, lm_ptr] : lms) {
      auto lm = lm_ptr.lock();
      if (lm) {
        if (lm->is_initialized_) {
          pc_source.emplace_back(lm->GetLandmarkPosition());
        }
      }
    }
    if (!pc_source.empty()) {
      vslam::viewer::PointCloudPrimitive pcp("pc", pc_source);
      viewer.PushPrimitive(pcp);
    }

    // Draw camera position
    auto frame_ptr =
        threadsafe_map_database_ptr
            ->GetFrame(threadsafe_map_database_ptr->latest_frame_id_)
            .lock();
    if (frame_ptr) {
      if (frame_ptr->is_keyframe_) {
        viewer.PushPrimitive(vslam::viewer::CoordinateSystemPrimitive(
            "kf" + std::to_string(frame_ptr->frame_id_),
            frame_ptr->GetCameraPose()));
      } else {
        viewer.PushPrimitive(vslam::viewer::CoordinateSystemPrimitive(
            "current_cam", frame_ptr->GetCameraPose()));
      }
      estimated_trajectory.emplace_back(
          frame_ptr->GetCameraPose().translation());
    }
    vslam::viewer::PointCloudPrimitive trajectory_primitive(
        "traj", estimated_trajectory);
    viewer.PushPrimitive(trajectory_primitive);

    cv::imshow("First", vis);
    //    if ((counter == 0) || (frame_ptr->is_keyframe_)) {
    if ((counter == 0)) {
      cv::waitKey(0);
    } else {
      auto c = cv::waitKey(30);
      if (c == 32) {
        is_update = !is_update;
      }
    }

    counter++;
  }

  return 0;
}
