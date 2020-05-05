//
// Created by ery on 2020/05/05.
//

#include <fmt/format.h>

#include "EurocKimeraDataProvider.hpp"
#include "FeatureDetectorShiTomasi.hpp"
#include "FeatureTrackerLucasKanade.hpp"
#include "KimeraFrontend.hpp"

int main() {
  std::string path_to_euroc = "/home/ery/Downloads/V1_01_easy";
  //  std::string path_to_euroc = "/home/ery/Downloads/V2_01_easy";
  vslam::dataprovider::EurocKimeraDataProvider euroc_kimera_data_provider(
      path_to_euroc);

  // Build detector
  vslam::feature::FeatureDetectorShiTomasi shi_tomasi_detector(4, 4, 300, 2.0);

  // Build tracker
  vslam::feature::FeatureTrackerLucasKanade kl_tracker(30, 0.01, 15, 3);

  vslam::data::FrameSharedPtr prev_frame = nullptr;
  vslam::FeatureAgeDatabase prev_feature_age;
  vslam::FeaturePositionDatabase prev_feature_position;

  vslam::frontend::KimeraFrontendInput prev_input;
  bool is_initialized = false;

  bool is_reach_the_last = false;
  uint64_t counter = 0;
  while (!is_reach_the_last) {
    auto input = euroc_kimera_data_provider.GetInput();
    if (input == std::nullopt) {
      is_reach_the_last = true;
      continue;
    }

    // detect
    if (!is_initialized) {
      shi_tomasi_detector.UpdateDetection(
          prev_feature_position, prev_feature_age, input.value().frame_);
      is_initialized = true;
    } else {
      // 10 Frameごとに特徴点を追加する
      if (counter++ % 10 == 0) {
        shi_tomasi_detector.UpdateDetection(
            prev_feature_position, prev_feature_age, input.value().frame_);
      }
      kl_tracker.Track(prev_feature_position,
                       prev_feature_age,
                       prev_input.frame_,
                       input.value().frame_);
    }

    // visualize
    cv::Mat vis;
    input.value().frame_.copyTo(vis);
    for (const auto& [id, pos] : prev_feature_position) {
      cv::circle(vis, cv::Point(pos[0], pos[1]), 1, cv::Scalar(255, 0, 0), 1);
    }
    // draw feature point number
    std::string str_feature_number =
        fmt::format("Features : {}", prev_feature_position.size());
    cv::putText(vis,
                str_feature_number,
                cv::Point(20, 20),
                CV_FONT_NORMAL,
                0.8,
                cv::Scalar(0, 0, 0),
                1,
                CV_AA);

    prev_input = input.value();

    cv::imshow("First", vis);
    cv::waitKey(10);
  }

  return 0;
}