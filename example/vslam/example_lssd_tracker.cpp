//
// Created by ery on 2020/06/08.
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

int main() {
  //  std::string path_to_euroc = "/home/ery/Downloads/V1_01_easy";
  //  std::string path_to_euroc = "/home/ery/Downloads/V2_01_easy";
  //  std::string path_to_euroc = "/home/ery/Downloads/MH_01_easy";
  //  vslam::dataprovider::EurocKimeraDataProviderRadialTangentialCameraModel
  //      euroc_kimera_data_provider(path_to_euroc);

  // EUROC
  //  std::string path_to_euroc =
  //      "/home/ery/subspace/docker_work/dataset/V1_01_easy";
  //  std::string path_to_calibfile =
  //      "/home/ery/subspace/docker_work/dataset/basalt_calib/euroc_calib/"
  //      "calib_results/calibration.json";

  std::string path_to_euroc =
      "/home/ery/subspace/docker_work/dataset/dataset-corridor1_512_16";
  std::string path_to_calibfile =
      "/home/ery/subspace/docker_work/dataset/basalt_calib/tumvi_calib_data/"
      "results/calibration.json";
  //
  vslam::dataprovider::EurocKimeraDataProvider euroc_kimera_data_provider(
      path_to_euroc, path_to_calibfile);

  //  std::string path_to_euroc =
  //      "/e/subspace/docker_work/dataset/fukuroi/camlog_2020-05-13-21-09-39/";
  //  std::string path_to_calibfile =
  //      "/e/subspace/docker_work/dataset/fukuroi/calib_result/calibration.json";
  //  std::string path_to_mask =
  //      "/e/subspace/docker_work/dataset/fukuroi/calib_result/vingette_0.png";
  //  vslam::dataprovider::EurocKimeraDataProvider euroc_kimera_data_provider(
  //      path_to_euroc, path_to_calibfile, path_to_mask);
  //
  //  spdlog::info(
  //      "Load dataset from:\n"
  //      "Dataset : {}\n"
  //      "Calibfile : {}",
  //      path_to_euroc,
  //      path_to_calibfile);

  // For output
  //  cv::VideoWriter video_writer("feature_tracking_and_detection.mp4",
  //                               cv::VideoWriter::fourcc('m','p','4', 'v'),
  //                               30,
  //                               euroc_kimera_data_provider.GetInput(0).value().frame_.size());

  auto kl_lssd_tracker_ptr =
      std::make_shared<vslam::feature::FeatureTrackerLSSDLucasKanade>(
          euroc_kimera_data_provider.GetInput(0)->frame_.size[0],
          euroc_kimera_data_provider.GetInput(0)->frame_.size[1],
          0,
          0,
          0,
          0,
          0);

  vslam::data::FrameSharedPtr prev_frame = nullptr;
  vslam::FeatureAgeDatabase prev_feature_age;
  vslam::FeaturePositionDatabase prev_feature_position;

  vslam::frontend::KimeraFrontendInputRadialTangentialCameraModel prev_input;
  bool is_initialized = false;

  bool is_update = false;

  bool is_reach_the_last = false;
  uint64_t counter = 0;
  while (!is_reach_the_last) {
    if (!is_update) {
      auto input = euroc_kimera_data_provider.GetInput();
      if (input == std::nullopt) {
        is_reach_the_last = true;
        continue;
      }

      kl_lssd_tracker_ptr->Track(input.value().frame_);
    }
  }

  return 0;
}
