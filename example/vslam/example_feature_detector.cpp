//
// Created by ery on 2020/05/03.
//

#include "EurocKimeraDataProviderRadialTangentialCameraModel.hpp"
#include "FeatureDetectorShiTomasi.hpp"

int main() {
  std::string path_to_euroc = "/home/ery/Downloads/V1_01_easy";
  //  std::string path_to_euroc = "/home/ery/Downloads/V2_01_easy";
  vslam::dataprovider::EurocKimeraDataProviderRadialTangentialCameraModel
      euroc_kimera_data_provider(path_to_euroc);

  // Build detector
  vslam::feature::FeatureDetectorShiTomasi shi_tomasi_detector(4, 4, 300, 1.0);

  vslam::data::FrameSharedPtr prev_frame = nullptr;
  vslam::FeatureAgeDatabase prev_feature_age;
  vslam::FeaturePositionDatabase prev_feature_position;

  bool is_reach_the_last = false;
  while (!is_reach_the_last) {
    auto input = euroc_kimera_data_provider.GetInput();
    if (input == std::nullopt) {
      is_reach_the_last = true;
      continue;
    }

    // detect
    shi_tomasi_detector.UpdateDetection(
        prev_feature_position, prev_feature_age, input.value().frame_);

    // visualize
    cv::Mat vis;
    input.value().frame_.copyTo(vis);
    for (const auto& [id, pos] : prev_feature_position) {
      cv::circle(vis, cv::Point(pos[0], pos[1]), 1, cv::Scalar(255, 0, 0), 1);
    }

    cv::imshow("First", vis);
    cv::waitKey(10);
  }

  return 0;
}