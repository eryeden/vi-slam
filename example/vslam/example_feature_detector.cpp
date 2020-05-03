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

    cv::imshow("First", input->frame_);
    cv::waitKey(10);
  }

  return 0;
}