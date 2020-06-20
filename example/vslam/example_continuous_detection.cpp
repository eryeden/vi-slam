//
// Created by ery on 2020/06/11.
//

#include <fmt/chrono.h>
#include <fmt/format.h>
#include <spdlog/spdlog.h>

#include <filesystem>

#include "ContinuousDetectorBackend.hpp"
#include "ContinuousDetectorFrontend.hpp"
#include "DataUtilities.hpp"
#include "EurocKimeraDataProvider.hpp"
#include "EurocKimeraDataProviderRadialTangentialCameraModel.hpp"
#include "FeatureDetectorANMS.hpp"
#include "FeatureDetectorShiTomasiBucketing.hpp"
#include "FeatureTrackerLSSDLucasKanade.hpp"
#include "FeatureTrackerLucasKanade.hpp"
#include "KittiKimeraDataProvider.hpp"
#include "LogDataOutput.hpp"
#include "Serializations.hpp"
#include "TumDataOutput.hpp"
#include "Verification.hpp"
#include "ViewerViz.hpp"

int main() {
  //  std::string path_to_euroc = "/home/ery/Downloads/V1_01_easy";
  //  std::string path_to_euroc = "/home/ery/Downloads/V2_01_easy";
  //  std::string path_to_euroc = "/home/ery/Downloads/MH_01_easy";
  //  vslam::dataprovider::EurocKimeraDataProviderRadialTangentialCameraModel
  //      euroc_kimera_data_provider(path_to_euroc);

  // EUROC
  //    std::string path_to_euroc =
  //        "/home/ery/subspace/docker_work/dataset/V1_01_easy";
  std::string path_to_euroc =
      "/home/ery/subspace/docker_work/dataset/V1_02_medium";
  //    std::string path_to_euroc =
  //        "/home/ery/subspace/docker_work/dataset/V1_03_difficult";
  //  std::string path_to_euroc =
  //      "/home/ery/subspace/docker_work/dataset/V2_01_easy";
  //  std::string path_to_euroc =
  //      "/home/ery/subspace/docker_work/dataset/MH_01_easy";
  //  std::string path_to_euroc =
  //      "/home/ery/subspace/docker_work/dataset/MH_02_easy";
  //  std::string path_to_euroc =
  //      "/home/ery/subspace/docker_work/dataset/MH_03_medium";
  //  std::string path_to_euroc =
  //      "/home/ery/subspace/docker_work/dataset/MH_04_difficult";

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
  vslam::Pose_t pose_body_T_sensor = euroc_kimera_data_provider.GetSensorPose();

  //  std::string path_to_euroc =
  //      "/e/subspace/docker_work/dataset/fukuroi/camlog_2020-05-13-21-09-39/";
  //  std::string path_to_calibfile =
  //      "/e/subspace/docker_work/dataset/fukuroi/calib_result/calibration.json";
  //  std::string path_to_mask =
  //            "/e/subspace/docker_work/dataset/fukuroi/calib_result/vingette_0.png";
  //  vslam::dataprovider::EurocKimeraDataProvider
  //      euroc_kimera_data_provider(
  //            path_to_euroc, path_to_calibfile, path_to_mask);
  //  vslam::Pose_t pose_body_T_sensor =
  //  euroc_kimera_data_provider.GetSensorPose();

  std::string path_to_kitti =
      "/home/ery/subspace/docker_work/dataset/data_odometry_gray/dataset/"
      "sequences/00";
  vslam::dataprovider::KittiKimeraDataProvider kitti_kimera_data_provider(
      path_to_kitti);

  /**
   * @brief Generate primitive instances
   */
  /// Map database
  auto threadsafe_map_database_ptr =
      std::make_shared<vslam::data::ThreadsafeMapDatabase>();

  /// Build detector
  auto detector_param = vslam::feature::FeatureDetectorANMS::Parameter();
  detector_param.max_feature_number_ = 150;
  detector_param.min_feature_distance_ = 10.0;  // 10.0;
  detector_param.detection_min_feature_distance_ = 0.5;

  auto anms_detector_ptr =
      std::make_shared<vslam::feature::FeatureDetectorANMS>(detector_param);

  //// Build tracker
  auto lssd_params = vslam::feature::FeatureTrackerLSSDLucasKanade::Parameter();
  lssd_params.optical_flow_max_recovered_dist_ = 1.0;  // 1.0;
  auto lssd_tracker_ptr =
      std::make_shared<vslam::feature::FeatureTrackerLSSDLucasKanade>(
          lssd_params);

  /// Build verification
  auto verification_params =
      vslam::verification::FeatureVerification5PointRANSAC::Parameter();
  verification_params.ransac_threshold_angle_rad_ =
      5.0 * M_PI / 180.0;  // 1 * M_PI / 180.0;
  auto verification_ptr =
      std::make_shared<vslam::verification::FeatureVerification5PointRANSAC>(
          verification_params);

  /**
   * @brief Frontend
   */
  //  vslam::frontend::KimeraFrontend::Parameter frontend_param;
  //  frontend_param.minimum_keyframe_interval_ = 0.1;
  //  frontend_param.low_keyframe_feature_number_ = 150;  // 250;
  //  vslam::frontend::KimeraFrontend
  //  kimera_frontend(threadsafe_map_database_ptr,
  //                                                  anms_detector_ptr,
  //                                                  lssd_tracker_ptr,
  //                                                  verification_ptr,
  //                                                  frontend_param);

  vslam::frontend::ContinuousDetectorFrontend::Parameter
      continuous_frontend_parameter;
  vslam::frontend::ContinuousDetectorFrontend continuous_detector_frontend(
      threadsafe_map_database_ptr,
      anms_detector_ptr,
      lssd_tracker_ptr,
      verification_ptr,
      continuous_frontend_parameter);

  /**
   * @brief Backend
   */
  vslam::backend::iSAM2Backend::Parameter isam2_backend_paramter;
  isam2_backend_paramter.keyframe_new_kf_keypoints_threshold_ = 0.7;
  isam2_backend_paramter.keyframe_min_frames_after_kf_ = 5;
  vslam::backend::ContinuousDetectorBackend continuous_detector_backend(
      threadsafe_map_database_ptr, isam2_backend_paramter);

  vslam::backend::BackendState backend_state =
      vslam::backend::BackendState::BootStrap;

  /**
   * @brief Log dumps
   */
  // Dump detailed log
  std::string path_to_output_log_dir =
      "/home/ery/subspace/docker_work/dataset/result/logs/";
  std::vector<vslam::data::InternalMaterials> internals;
  std::time_t current_time = std::time(nullptr);
  std::string current_log_directory_name =
      fmt::format("{:%Y-%m-%d-%H-%M-%S}", *std::localtime(&current_time));
  std::string current_log_output_directory =
      path_to_output_log_dir + "/" + current_log_directory_name;
  auto mkdir_result =
      std::filesystem::create_directory(current_log_output_directory);
  vslam::dataoutput::LogDataOutput log_data_output(
      current_log_output_directory);
  log_data_output.Dump(detector_param);
  log_data_output.Dump(lssd_params);
  //  log_data_output.Dump(frontend_param);
  log_data_output.Dump(isam2_backend_paramter);
  log_data_output.Dump(verification_params);
  // Dump trajectory
  std::string path_to_output_trajectory =
      current_log_output_directory + "/trajectory_body_frame.tum";
  vslam::dataoutput::TumDataOutput tum_data_output(path_to_output_trajectory);

  /**
   * @brief For visualization
   */
  auto previous_backend_state = backend_state;
  vslam::frontend::KimeraFrontendInputRadialTangentialCameraModel prev_input;
  bool is_update = true;
  bool is_reach_the_last = false;
  bool is_draw_initial_pose = true;
  bool is_draw_keyframe = true;
  uint64_t counter = 0;

  /**
   * @brief Generate viewer
   */
  // Generate viewer
  auto viewer = vslam::viewer::ViewerViz();
  // Feed drawing primitives
  viewer.PushPrimitive(vslam::viewer::CoordinateSystemPrimitive(
      "world_origin", vslam::Vec3_t(0, 0, 0), vslam::Quat_t::Identity()));
  vslam::EigenAllocatedVector<vslam::Pose_t> trajectory_keyframe;
  vslam::EigenAllocatedVector<vslam::Pose_t> trajectory_isam2;
  vslam::EigenAllocatedVector<vslam::Pose_t> trajectory_initialized;

  // Create a window and render primitives.
  viewer.LaunchViewer();

  while (!is_reach_the_last) {
    /**
     * @brief VSLAM Process
     */
    if (is_update) {
      auto input = euroc_kimera_data_provider.GetInput();
      //      auto input = kitti_kimera_data_provider.GetInput();
      if (input == std::nullopt) {
        is_reach_the_last = true;
        continue;
      }
      //      kimera_frontend.Feed(input.value());
      continuous_detector_frontend.Feed(input.value());
      //      backend_state = i_sam_2_backend.SpinOnce();
      backend_state = continuous_detector_backend.SpinOnce();

      /**
       * @brief Log
       */
      auto latest_frame_ptr =
          threadsafe_map_database_ptr
              ->GetFrame(threadsafe_map_database_ptr->latest_frame_id_)
              .lock();
      if (latest_frame_ptr) {
        /// Update internals
        latest_frame_ptr->internal_materials_ =
            vslam::utility::GenerateInternalsFromFrame(
                *latest_frame_ptr, threadsafe_map_database_ptr);
        latest_frame_ptr->internal_materials_.body_pose_ =
            pose_body_T_sensor.inverse() * latest_frame_ptr->GetCameraPose();
        log_data_output.Dump(counter, latest_frame_ptr->internal_materials_);
      }
      counter++;
    }

    auto latest_frame_ptr =
        threadsafe_map_database_ptr
            ->GetFrame(threadsafe_map_database_ptr->latest_frame_id_)
            .lock();
    if (latest_frame_ptr) {
      /**
       * @brief Visualize
       */
      /// Draw detected features
      cv::Mat vis;
      cv::cvtColor(
          continuous_detector_frontend.last_input_.frame_, vis, CV_GRAY2BGR);
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

        auto lm_ptr = threadsafe_map_database_ptr->GetLandmark(id).lock();
        if (lm_ptr) {
          if (lm_ptr->is_initialized_ && !(lm_ptr->is_outlier_)) {
            cv::circle(vis,
                       cv::Point(pos[0], pos[1]),
                       5,
                       cv::Scalar(0, 255, 0),
                       1,
                       CV_AA);
          }
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
        //        viewer.PushPrimitive(vslam::viewer::CoordinateSystemPrimitive(
        //            "kf" + std::to_string(latest_frame_ptr->frame_id_),
        //            pose_world_T_body));
        trajectory_keyframe.emplace_back(pose_world_T_body);
      } else {
        viewer.PushPrimitive(vslam::viewer::CoordinateSystemPrimitive(
            "current_cam", pose_world_T_body));
      }
      cv::imshow("Keypoints", vis);
      /// Draw trajectory
      trajectory_isam2.emplace_back(pose_world_T_body);
      trajectory_initialized.emplace_back(
          pose_body_T_sensor.inverse() *
          latest_frame_ptr->internal_materials_.camera_pose_initial_);
      if (is_draw_keyframe) {
        viewer.PushPrimitive(vslam::viewer::TrajectoryPrimitive(
            "traj_keyframe",
            trajectory_keyframe,
            vslam::Vec3_t(0, 0, 255),
            vslam::viewer::TrajectoryPrimitive::FRAMES));
      } else {
        viewer.PushPrimitive(vslam::viewer::TrajectoryPrimitive(
            "traj_keyframe",
            {pose_world_T_body},
            vslam::Vec3_t(0, 0, 255),
            vslam::viewer::TrajectoryPrimitive::FRAMES));
      }
      viewer.PushPrimitive(vslam::viewer::TrajectoryPrimitive(
          "traj_isam2", trajectory_isam2, vslam::Vec3_t(0, 0, 255)));
      if (is_draw_initial_pose) {
        viewer.PushPrimitive(
            vslam::viewer::TrajectoryPrimitive("traj_initial",
                                               trajectory_initialized,
                                               vslam::Vec3_t(0, 255, 255)));
      } else {
        viewer.PushPrimitive(vslam::viewer::TrajectoryPrimitive(
            "traj_initial", {}, vslam::Vec3_t(0, 255, 255)));
      }

      /// Draw estimated landmark position
      vslam::EigenAllocatedVector<vslam::Vec3_t> pc_source;
      auto lms = threadsafe_map_database_ptr->GetAllLandmarks();
      for (const auto& [id, lm_ptr] : lms) {
        auto lm = lm_ptr.lock();
        if (lm) {
          if (lm->is_initialized_) {
            pc_source.emplace_back(pose_body_T_sensor.inverse() *
                                   lm->GetLandmarkPosition());
            // pc_source.emplace_back(lm->GetLandmarkPosition());
          }
        }
      }
      if (!pc_source.empty()) {
        vslam::viewer::PointCloudPrimitive pcp(
            "pc", pc_source, false, {{255, 255, 255}});
        viewer.PushPrimitive(pcp);
      }

      /// Draw in-use landmark position
      vslam::EigenAllocatedVector<vslam::Vec3_t> pc_source_inuse;
      for (const auto id : latest_frame_ptr->observing_feature_id_) {
        auto lm_ptr = threadsafe_map_database_ptr->GetLandmark(id).lock();
        if (lm_ptr) {
          if (lm_ptr->is_initialized_) {
            pc_source_inuse.emplace_back(pose_body_T_sensor.inverse() *
                                         lm_ptr->GetLandmarkPosition());
          }
        }
      }
      if (!pc_source_inuse.empty()) {
        vslam::viewer::PointCloudPrimitive pcp(
            "pc_inuse", pc_source_inuse, false, {{255, 0, 255}});
        viewer.PushPrimitive(pcp);
      }

      /// Wait
      if ((counter == 0) || (previous_backend_state != backend_state)) {
        cv::waitKey(0);
      } else {
        auto c = cv::waitKey(30);
        if (c == 32) {
          is_update = !is_update;
        } else if (c == 'q') {
          is_draw_initial_pose = !is_draw_initial_pose;
        } else if (c == 'w') {
          is_draw_keyframe = !is_draw_keyframe;
        }
      }
    }

    previous_backend_state = backend_state;
  }

  return 0;
}
