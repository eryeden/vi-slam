//
// Created by ery on 2020/07/04.
//

#include <fmt/chrono.h>
#include <fmt/format.h>
#include <spdlog/spdlog.h>

#include <filesystem>
#include <iostream>
#include <sstream>

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
#include "Landmark.hpp"
#include "LogDataOutput.hpp"
#include "Serializations.hpp"
#include "TumDataOutput.hpp"
#include "Verification.hpp"
#include "ViewerViz.hpp"
#include "cxxopts.hpp"

int main(int argc, char** argv) {
  /**
   * @brief Set defines
   */
  std::string path_to_project_root = PROJECT_ROOT_DIR;

  /**
   * @brief Parse input arguments.
   */
  cxxopts::Options arg_options("VSLAM",
                               "This is a test program for Visual SLAM.");
  arg_options.add_options()(
      "e,euroc_config",
      "Path to the euroc config file.",
      cxxopts::value<std::string>()->default_value(
          path_to_project_root + "/params/EurocKimeraDataProvider.json"))(
      "r,result",
      "Path to output directory.",
      cxxopts::value<std::string>()->default_value(path_to_project_root +
                                                   "/results"))(
      "d,date", "Is generate date named new directory ?")(
      "f,fail_to_exit", "Is exit program when it fail ?")("h,help",
                                                          "Print usage");
  auto arg_result = arg_options.parse(argc, argv);
  if (arg_result.count("help")) {
    std::cout << arg_options.help() << std::endl;
    exit(0);
  }
  bool is_generate_dated_directory = false;
  if (arg_result.count("date")) {
    is_generate_dated_directory = true;
  }
  bool is_exit_when_fail = false;
  if (arg_result.count("fail_to_exit")) {
    is_exit_when_fail = true;
  }
  auto path_to_euroc_config = arg_result["euroc_config"].as<std::string>();
  auto path_to_output_directory = arg_result["result"].as<std::string>();

  /**
   * @brief Parse config files
   */
  vslam::dataprovider::EurocKimeraDataProvider::Parameter
      euroc_datadrovider_param;
  {
    //入力される文字列受け皿
    std::stringstream input_stream;
    //ファイル入力ストリーム作成
    std::ifstream input_file(path_to_euroc_config, std::ios::in);
    //入力データを全部文字列streamに投げる
    input_stream << input_file.rdbuf();
    cereal::JSONInputArchive json_input_archive(input_stream);
    json_input_archive(euroc_datadrovider_param);
  }

  /**
   * @brief Generate dataprovider
   */
  vslam::dataprovider::EurocKimeraDataProvider euroc_kimera_data_provider(
      euroc_datadrovider_param);
  vslam::Pose_t pose_body_T_sensor = euroc_kimera_data_provider.GetSensorPose();

  /**
   * @brief Generate primitive instances
   */
  /// Map database
  auto threadsafe_map_database_ptr =
      std::make_shared<vslam::data::ThreadsafeMapDatabase>();

  /// Build detector
  auto detector_param = vslam::feature::FeatureDetectorANMS::Parameter();
  detector_param.max_feature_number_ = 300;  // 150
  detector_param.detection_feature_number_ = 600;
  detector_param.min_feature_distance_ = 5.0;            // 10.0;
  detector_param.detection_min_feature_distance_ = 0.5;  // 0.5

  auto anms_detector_ptr =
      std::make_shared<vslam::feature::FeatureDetectorANMS>(detector_param);

  //// Build tracker
  auto lssd_params = vslam::feature::FeatureTrackerLSSDLucasKanade::Parameter();
  lssd_params.optical_flow_max_recovered_dist_ = 0.5;  // 1.0;　0.5
  auto lssd_tracker_ptr =
      std::make_shared<vslam::feature::FeatureTrackerLSSDLucasKanade>(
          lssd_params);

  /// Build verification
  auto verification_params =
      vslam::verification::FeatureVerification5PointRANSAC::Parameter();
  verification_params.ransac_threshold_angle_rad_ =
      0.1 * M_PI / 180.0;  // 1 * M_PI / 180.0;
  auto verification_ptr =
      std::make_shared<vslam::verification::FeatureVerification5PointRANSAC>(
          verification_params);

  /**
   * @brief Frontend
   */
  vslam::frontend::ContinuousDetectorFrontend::Parameter
      continuous_frontend_parameter;
  continuous_frontend_parameter.keyframe_min_frames_after_kf_ = 5;
  continuous_frontend_parameter.keyframe_new_kf_keypoints_threshold_ = 0.5;
  continuous_frontend_parameter.keyframe_new_kf_keypoints_minimum_threshold_ =
      0.2;
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
  isam2_backend_paramter.isam2_iteration_number_ = 5;
  isam2_backend_paramter.triangulation_minimum_parallax_threshold_ =
      0.03;  // 0.024
  vslam::backend::ContinuousDetectorBackend continuous_detector_backend(
      threadsafe_map_database_ptr, isam2_backend_paramter);

  vslam::backend::BackendState backend_state =
      vslam::backend::BackendState::BootStrap;

  /**
   * @brief Log dumps
   */
  // Dump detailed log

  std::string path_to_output_log_dir = path_to_output_directory;
  std::vector<vslam::data::InternalMaterials> internals;
  std::string current_log_output_directory = path_to_output_log_dir;
  if (is_generate_dated_directory) {
    std::time_t current_time = std::time(nullptr);
    std::string current_log_directory_name =
        fmt::format("{:%Y-%m-%d-%H-%M-%S}", *std::localtime(&current_time));
    current_log_output_directory =
        path_to_output_log_dir + "/" + current_log_directory_name;
    auto mkdir_result =
        std::filesystem::create_directory(current_log_output_directory);
  }
  vslam::dataoutput::LogDataOutput log_data_output(
      current_log_output_directory);
  log_data_output.Dump(detector_param);
  log_data_output.Dump(lssd_params);
  //  log_data_output.Dump(frontend_param);
  log_data_output.Dump(isam2_backend_paramter);
  log_data_output.Dump(verification_params);
  log_data_output.Dump(euroc_datadrovider_param);
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
  bool is_frame_to_frame = false;
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
  vslam::EigenAllocatedVector<vslam::Pose_t> trajectory_motion_only_ba;
  // For Frame visualization
  cv::Mat vis, vis_pose_init, vis_triangulate, vis_takeover;

  // Create a window and render primitives.
  viewer.LaunchViewer();

  while (!is_reach_the_last) {
    /**
     * @brief VSLAM Process
     */
    if (is_update) {
      auto input = euroc_kimera_data_provider.GetInput();
      //            auto input = kitti_kimera_data_provider.GetInput();
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
      cv::cvtColor(
          continuous_detector_frontend.last_input_.frame_, vis, CV_GRAY2BGR);
      vis.copyTo(vis_pose_init);
      if (latest_frame_ptr->is_keyframe_) {
        vis.copyTo(vis_takeover);
        vis.copyTo(vis_triangulate);
      }

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
        //        cv::putText(vis,
        //                    fmt::format("{}", id),
        //                    cv::Point(pos[0], pos[1]),
        //                    CV_FONT_HERSHEY_PLAIN,
        //                    0.8,
        //                    cv::Scalar(0, 0, 0));

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
          if (lm_ptr->is_nearby_) {
            cv::circle(vis,
                       cv::Point(pos[0], pos[1]),
                       6,
                       cv::Scalar(0, 255, 255),
                       1,
                       CV_AA);
            cv::putText(vis,
                        fmt::format("Nearby: {}", id),
                        cv::Point(pos[0], pos[1]),
                        CV_FONT_HERSHEY_PLAIN,
                        0.8,
                        cv::Scalar(0, 0, 0));
          }
        }

        if (latest_frame_ptr->internal_materials_.pose_initialization_landmarks_
                .count(id) != 0) {
          cv::circle(vis_pose_init,
                     cv::Point(pos[0], pos[1]),
                     5,
                     cv::Scalar(0, 255, 0),
                     1,
                     CV_AA);
        }
        if (latest_frame_ptr->internal_materials_.take_over_landmarks_.count(
                id) != 0) {
          cv::circle(vis_takeover,
                     cv::Point(pos[0], pos[1]),
                     5,
                     cv::Scalar(0, 255, 0),
                     1,
                     CV_AA);
        }
        if (latest_frame_ptr->internal_materials_.triangulated_landmarks_.count(
                id) != 0) {
          cv::circle(vis_triangulate,
                     cv::Point(pos[0], pos[1]),
                     5,
                     cv::Scalar(0, 255, 0),
                     1,
                     CV_AA);
          cv::putText(vis_triangulate,
                      fmt::format("P:{:.3},L:{:.3}",
                                  lm_ptr->triangulate_parallax_angle_,
                                  lm_ptr->triangulate_baseline_length_),
                      cv::Point(pos[0], pos[1]),
                      CV_FONT_HERSHEY_PLAIN,
                      0.8,
                      cv::Scalar(0, 0, 0),
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
        //        viewer.PushPrimitive(vslam::viewer::CoordinateSystemPrimitive(
        //            "kf" + std::to_string(latest_frame_ptr->frame_id_),
        //            pose_world_T_body));
        trajectory_keyframe.emplace_back(pose_world_T_body);
      } else {
        viewer.PushPrimitive(vslam::viewer::CoordinateSystemPrimitive(
            "current_cam", pose_world_T_body));
      }
      cv::imshow("Keypoints", vis);
      cv::imshow("Triangulated_lm", vis_triangulate);
      cv::imshow("Takeover_lm", vis_takeover);
      cv::imshow("Pose_init_lm", vis_pose_init);

      /// Draw trajectory
      trajectory_isam2.emplace_back(pose_world_T_body);
      trajectory_initialized.emplace_back(
          pose_body_T_sensor.inverse() *
          latest_frame_ptr->internal_materials_.camera_pose_initial_);
      trajectory_motion_only_ba.emplace_back(
          pose_body_T_sensor.inverse() *
          latest_frame_ptr->internal_materials_.camera_pose_optimized_);

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
        viewer.PushPrimitive(vslam::viewer::TrajectoryPrimitive(
            "traj_opt", trajectory_motion_only_ba, vslam::Vec3_t(255, 0, 255)));
      } else {
        viewer.PushPrimitive(vslam::viewer::TrajectoryPrimitive(
            "traj_initial", {}, vslam::Vec3_t(0, 255, 255)));
        viewer.PushPrimitive(vslam::viewer::TrajectoryPrimitive(
            "traj_opt", {}, vslam::Vec3_t(255, 0, 255)));
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
            "pc_inuse", pc_source_inuse, true, {{255, 0, 255}});
        viewer.PushPrimitive(pcp);
      }

      /// Draw triangulated landmark position
      vslam::EigenAllocatedVector<vslam::Vec3_t> pc_source_triangulated;
      for (const auto& [lm_id, lm] :
           latest_frame_ptr->internal_materials_.triangulated_landmarks_) {
        pc_source_triangulated.emplace_back(pose_body_T_sensor.inverse() *
                                            lm.GetLandmarkPosition());
      }
      if (!pc_source_triangulated.empty()) {
        vslam::viewer::PointCloudPrimitive pcp(
            "pc_triangulated", pc_source_triangulated, true, {{0, 255, 255}});
        viewer.PushPrimitive(pcp);
      }

      /// Wait
      int32_t wait_time = 10;
      if ((counter == 0) || (previous_backend_state != backend_state) ||
          is_frame_to_frame) {
        wait_time = 0;
      }

      auto c = cv::waitKey(wait_time);
      if (c == 32) {
        is_update = !is_update;
      } else if (c == 'q') {
        is_draw_initial_pose = !is_draw_initial_pose;
      } else if (c == 'w') {
        is_draw_keyframe = !is_draw_keyframe;
      } else if (c == 'f') {
        is_frame_to_frame = !is_frame_to_frame;
      }

      if (is_exit_when_fail) {
        if (backend_state == vslam::backend::BackendState::SolverException) {
          /**
           * @brief Store all LM and
           */
          std::unordered_map<vslam::database_index_t, vslam::data::Landmark>
              output_landmarks;
          for (const auto& [lm_id, lm_weak] :
               threadsafe_map_database_ptr->GetAllLandmarks()) {
            auto lm_ptr = lm_weak.lock();
            if (lm_ptr) {
              output_landmarks.insert(
                  std::pair<vslam::database_index_t, vslam::data::Landmark>(
                      lm_id, *lm_ptr));
            }
          }
          log_data_output.Dump(output_landmarks);

          return 0;
        }
      }
    }

    previous_backend_state = backend_state;
  }

  /**
   * @brief Store all LM and
   */
  std::unordered_map<vslam::database_index_t, vslam::data::Landmark>
      output_landmarks;
  for (const auto& [lm_id, lm_weak] :
       threadsafe_map_database_ptr->GetAllLandmarks()) {
    auto lm_ptr = lm_weak.lock();
    if (lm_ptr) {
      output_landmarks.insert(
          std::pair<vslam::database_index_t, vslam::data::Landmark>(lm_id,
                                                                    *lm_ptr));
    }
  }
  log_data_output.Dump(output_landmarks);

  return 0;
}