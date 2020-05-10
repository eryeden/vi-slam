//
// Created by ery on 2020/05/06.
//

#include <fmt/format.h>

#include "EurocKimeraDataProvider.hpp"
#include "FeatureDetectorShiTomasi.hpp"
#include "FeatureTrackerLucasKanade.hpp"
#include "KimeraFrontend.hpp"
#include "Verification.hpp"

int main() {
  std::string path_to_euroc = "/home/ery/Downloads/V1_01_easy";
  //  std::string path_to_euroc = "/home/ery/Downloads/V2_01_easy";
  //  std::string path_to_euroc = "/home/ery/Downloads/MH_01_easy";
  vslam::dataprovider::EurocKimeraDataProvider euroc_kimera_data_provider(
      path_to_euroc);

  // For output
  //  cv::VideoWriter video_writer("feature_tracking_and_detection.mp4",
  //                               cv::VideoWriter::fourcc('m','p','4', 'v'),
  //                               30,
  //                               euroc_kimera_data_provider.GetInput(0).value().frame_.size());

  // Map database
  auto threadsafe_map_database_ptr =
      std::make_shared<vslam::data::ThreadsafeMapDatabase>();

  // Build detector
  auto shi_tomasi_detector_ptr =
      std::make_shared<vslam::feature::FeatureDetectorShiTomasi>(
          3, 3, 300, 20.0);

  // Build tracker
  auto kl_tracker_ptr =
      std::make_shared<vslam::feature::FeatureTrackerLucasKanade>(
          30, 0.01, 15, 3);

  // Build verification
  auto verification_ptr =
      std::make_shared<vslam::verification::FeatureVerification5PointRANSAC>(
          1.0 * M_PI / 180.0, 150, 0.99);

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
  //      std::shared_ptr<vslam::feature::FeatureDetectorShiTomasi>(
  //          &shi_tomasi_detector),
  //      std::shared_ptr<vslam::feature::FeatureTrackerLucasKanade>(&kl_tracker),
  //      100.0,
  //      100);

  vslam::frontend::KimeraFrontend kimera_frontend(threadsafe_map_database_ptr,
                                                  shi_tomasi_detector_ptr,
                                                  kl_tracker_ptr,
                                                  verification_ptr,
                                                  10.0,
                                                  100);

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

    //    if (counter > 100) {
    //      is_reach_the_last = true;
    //      continue;
    //    }

    //    // detect
    //    if (!is_initialized) {
    //      shi_tomasi_detector.UpdateDetection(
    //          prev_feature_position, prev_feature_age, input.value().frame_);
    //      is_initialized = true;
    //    } else {
    //      // 10 Frameごとに特徴点を追加する
    //      if (counter++ % 10 == 0) {
    //        shi_tomasi_detector.UpdateDetection(
    //            prev_feature_position, prev_feature_age,
    //            input.value().frame_);
    //      }
    //      kl_tracker.Track(prev_feature_position,
    //                       prev_feature_age,
    //                       prev_input.frame_,
    //                       input.value().frame_);
    //    }

    kimera_frontend.Feed(input.value());

    // visualize
    cv::Mat vis;
    kimera_frontend.last_input_.frame_.copyTo(vis);

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
    //
    //    prev_input = input.value();

    counter++;

    cv::imshow("First", vis);
    //    video_writer << vis;
    cv::waitKey(20);
  }

  return 0;
}