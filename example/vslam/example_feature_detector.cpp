//
// Created by ery on 2020/05/03.
//

#include "EurocKimeraDataProvider.hpp"
#include "FeatureDetector.hpp"

int main() {
  std::string path_to_euroc = "/home/ery/Downloads/V1_01_easy";
  vslam::dataprovider::EurocKimeraDataProvider euroc_kimera_data_provider(
      path_to_euroc);

  bool is_reach_the_last = false;
  while (!is_reach_the_last) {
    auto input = euroc_kimera_data_provider.GetInput();
    if (input == std::nullopt) {
      is_reach_the_last = true;
      continue;
    }

    // detect
    auto frame = vslam::feature::DetectShiTomasiCorners(
        vslam::data::FrameSharedPtr(), input.value().frame_, 5, 5, 300);

    // visualize
    cv::Mat vis;
    //    cv::cvtColor(input.value().frame_, vis, CV_GRAY2BGR);
    input.value().frame_.copyTo(vis);
    for (const auto& [id, pos] : frame.observing_feature_point_in_device_) {
      cv::circle(vis, cv::Point(pos[0], pos[1]), 1, cv::Scalar(255, 0, 0), 1);
    }

    cv::imshow("First", vis);
    cv::waitKey(10);
  }

  return 0;
}