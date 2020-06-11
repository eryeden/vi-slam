//
// Created by ery on 2020/06/11.
//

#include <fmt/format.h>
#include <spdlog/spdlog.h>

#include "DataUtilities.hpp"
#include "EurocKimeraDataProvider.hpp"
#include "EurocKimeraDataProviderRadialTangentialCameraModel.hpp"
#include "FeatureDetectorANMS.hpp"
#include "FeatureDetectorShiTomasiBucketing.hpp"
#include "FeatureTrackerLSSDLucasKanade.hpp"
#include "FeatureTrackerLucasKanade.hpp"
#include "KimeraFrontend.hpp"
#include "Serializations.hpp"
#include "TumDataOutput.hpp"
#include "Verification.hpp"
#include "ViewerViz.hpp"
#include "iSAM2Backend.hpp"

int main() {
  //  std::string path_to_euroc = "/home/ery/Downloads/V1_01_easy";
  //  std::string path_to_euroc = "/home/ery/Downloads/V2_01_easy";
  //  std::string path_to_euroc = "/home/ery/Downloads/MH_01_easy";
  //  vslam::dataprovider::EurocKimeraDataProviderRadialTangentialCameraModel
  //      euroc_kimera_data_provider(path_to_euroc);

  // EUROC
  std::string path_to_euroc =
      "/home/ery/subspace/docker_work/dataset/V1_01_easy";
  //    std::string path_to_euroc =
  //              "/home/ery/subspace/docker_work/dataset/V2_01_easy";
  //    std::string path_to_euroc =
  //        "/home/ery/subspace/docker_work/dataset/MH_01_easy";
  std::string path_to_calibfile =
      "/home/ery/subspace/docker_work/dataset/basalt_calib/euroc_calib/"
      "calib_results/calibration.json";

  //        std::string path_to_euroc =
  //            "/home/ery/subspace/docker_work/dataset/dataset-corridor1_512_16";
  //        std::string path_to_calibfile =
  //            "/home/ery/subspace/docker_work/dataset/basalt_calib/tumvi_calib_data/"
  //            "results/calibration.json";

  vslam::dataprovider::EurocKimeraDataProvider euroc_kimera_data_provider(
      path_to_euroc, path_to_calibfile);
  vslam::Pose_t pose_body_T_sensor = euroc_kimera_data_provider.GetSensorPose();

  //      std::string path_to_euroc =
  //          "/e/subspace/docker_work/dataset/fukuroi/camlog_2020-05-13-21-09-39/";
  //      std::string path_to_calibfile =
  //          "/e/subspace/docker_work/dataset/fukuroi/calib_result/calibration.json";
  //      std::string path_to_mask =
  //          "/e/subspace/docker_work/dataset/fukuroi/calib_result/vingette_0.png";
  //      vslam::dataprovider::EurocKimeraDataProvider
  //      euroc_kimera_data_provider(
  //          path_to_euroc, path_to_calibfile, path_to_mask);

  std::string path_to_output_trajectory =
      "/home/ery/subspace/docker_work/dataset/result/out1.tum";
  std::string path_to_output_log_dir =
      "/home/ery/subspace/docker_work/dataset/result/logs/";
  vslam::dataoutput::TumDataOutput tum_data_output(path_to_output_trajectory);

  /**
   * @brief Generate primitive instances
   */
  /// Map database
  auto threadsafe_map_database_ptr =
      std::make_shared<vslam::data::ThreadsafeMapDatabase>();

  /// Build detector
  auto detector_param = vslam::feature::FeatureDetectorANMS::Parameter();
  detector_param.max_feature_number_ = 300;
  detector_param.min_feature_distance_ = 10.0;
  auto anms_detector_ptr =
      std::make_shared<vslam::feature::FeatureDetectorANMS>(detector_param);

  //// Build tracker
  auto lssd_params = vslam::feature::FeatureTrackerLSSDLucasKanade::Parameter();
  lssd_params.optical_flow_max_recovered_dist_ = 1.0;
  auto lssd_tracker_ptr =
      std::make_shared<vslam::feature::FeatureTrackerLSSDLucasKanade>(
          lssd_params);

  /// Build verification
  auto verification_params =
      vslam::verification::FeatureVerification5PointRANSAC::Parameter();
  verification_params.ransac_threshold_angle_rad_ = 1 * M_PI / 180.0;
  auto verification_ptr =
      std::make_shared<vslam::verification::FeatureVerification5PointRANSAC>(
          verification_params);

  /**
   * @brief Frontend
   */
  vslam::frontend::KimeraFrontend kimera_frontend(
      threadsafe_map_database_ptr,
      //                                                  shi_tomasi_detector_ptr,
      anms_detector_ptr,
      //      kl_tracker_ptr,
      lssd_tracker_ptr,
      verification_ptr,
      5.0,
      200);

  /**
   * @brief Backend
   */
  vslam::backend::iSAM2Backend i_sam_2_backend(threadsafe_map_database_ptr);
  vslam::backend::BackendState backend_state =
      vslam::backend::BackendState::BootStrap;

  /**
   * @brief Log dumps
   */
  std::vector<vslam::data::InternalMaterials> internals;

  /**
   * @brief For visualization
   */
  auto previous_backend_state = backend_state;
  vslam::frontend::KimeraFrontendInputRadialTangentialCameraModel prev_input;
  bool is_update = true;
  bool is_reach_the_last = false;
  uint64_t counter = 0;

  /**
   * @brief Generate viewer
   */
  // Generate viewer
  auto viewer = vslam::viewer::ViewerViz();
  // Feed drawing primitives
  viewer.PushPrimitive(vslam::viewer::CoordinateSystemPrimitive(
      "world_origin", vslam::Vec3_t(0, 0, 0), vslam::Quat_t::Identity()));
  vslam::EigenAllocatedVector<vslam::Vec3_t> estimated_trajectory;
  // Create a window and render primitives.
  viewer.LaunchViewer();

  while (!is_reach_the_last) {
    /**
     * @brief VSLAM Process
     */
    if (is_update) {
      auto input = euroc_kimera_data_provider.GetInput();
      if (input == std::nullopt) {
        is_reach_the_last = true;
        continue;
      }
      kimera_frontend.Feed(input.value());
      backend_state = i_sam_2_backend.SpinOnce();
    }

    auto latest_frame_ptr =
        threadsafe_map_database_ptr
            ->GetFrame(threadsafe_map_database_ptr->latest_frame_id_)
            .lock();
    if (latest_frame_ptr) {
      /**
       * @brief Log
       */
      /// Update internals
      latest_frame_ptr->internal_materials_ =
          vslam::utility::GenerateInternalsFromFrame(
              *latest_frame_ptr, threadsafe_map_database_ptr);
      //      internals.emplace_back(latest_frame_ptr->internal_materials_);
      std::string path_to_output_log =
          fmt::format("{}/frame_{}.json", path_to_output_log_dir, counter);
      std::ofstream log_output_stream(path_to_output_log, std::ios::out);
      {
        cereal::JSONOutputArchive json_output_archive(log_output_stream);
        json_output_archive(latest_frame_ptr->internal_materials_);
      }

      /**
       * @brief Visualize
       */
      /// Draw detected features
      cv::Mat vis;
      cv::cvtColor(kimera_frontend.last_input_.frame_, vis, CV_GRAY2BGR);
      for (const auto& [id, pos] :
           latest_frame_ptr->observing_feature_point_in_device_) {
        int32_t landmark_age = latest_frame_ptr->feature_point_age_.at(id);
        if (landmark_age <= 1) {
          cv::circle(vis,
                     cv::Point(pos[0], pos[1]),
                     2,
                     cv::Scalar(0, 0, 255),
                     1,
                     CV_AA);
        } else if (landmark_age == 2) {
          cv::circle(vis,
                     cv::Point(pos[0], pos[1]),
                     3,
                     cv::Scalar(255, 0, 0),
                     1,
                     CV_AA);
        } else if (landmark_age > 2) {
          cv::circle(vis,
                     cv::Point(pos[0], pos[1]),
                     3,
                     cv::Scalar(0, 255, 0),
                     1,
                     CV_AA);
        }
      }
      /// draw feature point number
      std::string str_feature_number = fmt::format(
          "Features : {}",
          latest_frame_ptr->observing_feature_point_in_device_.size());
      cv::putText(vis,
                  str_feature_number,
                  cv::Point(20, 20),
                  CV_FONT_NORMAL,
                  0.8,
                  cv::Scalar(0, 0, 0),
                  1,
                  CV_AA);

      /**
       * @brief 3D Visulization
       */
      auto pose_world_T_body =
          pose_body_T_sensor.inverse() * latest_frame_ptr->GetCameraPose();
      tum_data_output.SavePose(latest_frame_ptr->timestamp_, pose_world_T_body);
      /// Draw estiamted body position
      if (latest_frame_ptr->is_keyframe_) {
        viewer.PushPrimitive(vslam::viewer::CoordinateSystemPrimitive(
            "kf" + std::to_string(latest_frame_ptr->frame_id_),
            pose_world_T_body));
      } else {
        viewer.PushPrimitive(vslam::viewer::CoordinateSystemPrimitive(
            "current_cam", pose_world_T_body));
      }
      cv::imshow("Keypoints", vis);
      /// Draw trajectory
      estimated_trajectory.emplace_back(pose_world_T_body.translation());
      vslam::viewer::PointCloudPrimitive trajectory_primitive(
          "traj", estimated_trajectory, false, {vslam::Vec3_t(0, 0, 255)});
      viewer.PushPrimitive(trajectory_primitive);
      /// Draw estimated landmark position
      vslam::EigenAllocatedVector<vslam::Vec3_t> pc_source;
      auto lms = threadsafe_map_database_ptr->GetAllLandmarks();
      for (const auto& [id, lm_ptr] : lms) {
        auto lm = lm_ptr.lock();
        if (lm) {
          if (lm->is_initialized_) {
            pc_source.emplace_back(pose_body_T_sensor.inverse() *
                                   lm->GetLandmarkPosition());
            //          pc_source.emplace_back(lm->GetLandmarkPosition());
          }
        }
      }
      if (!pc_source.empty()) {
        vslam::viewer::PointCloudPrimitive pcp(
            "pc", pc_source, false, {{255, 255, 255}});
        viewer.PushPrimitive(pcp);
      }

      /// Wait
      if ((counter == 0) || (previous_backend_state != backend_state)) {
        cv::waitKey(0);
      } else {
        auto c = cv::waitKey(30);
        if (c == 32) {
          is_update = !is_update;
        }
      }
    }

    previous_backend_state = backend_state;
    counter++;
  }

  return 0;
}
