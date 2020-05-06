//
// Created by ery on 2020/05/03.
//

#include "FeatureDetectorShiTomasi.hpp"

#include <spdlog/spdlog.h>

#include "kdtree/KDTree.hpp"

std::vector<cv::Rect2f> vslam::feature::utility::GenerateGrid(
    const cv::Size& size,
    int32_t division_number_col,
    int32_t division_number_row) {
  std::vector<cv::Rect2f> grid_def(0);
  grid_def.reserve(division_number_col * division_number_row);

  // 画像をグリッドで分割して均等な分布になるようにする
  double dx, dy;
  dx = static_cast<double>(size.width) /
       static_cast<double>(division_number_col);
  dy = static_cast<double>(size.height) /
       static_cast<double>(division_number_row);

  for (int32_t x = 0; x < division_number_col; x++) {
    for (int32_t y = 0; y < division_number_row; y++) {
      cv::Point2f left_up(x * dx, y * dy);
      cv::Point2f right_down = left_up + cv::Point2f(dx, dy);
      grid_def.emplace_back(cv::Rect2f(left_up, right_down));
    }
  }

  return grid_def;
}

vslam::feature::FeatureDetectorShiTomasi::FeatureDetectorShiTomasi(
    int32_t division_number_row,
    int32_t division_number_col,
    int32_t max_feature_number,
    double min_feature_distance)
    : max_feature_index_(0),
      division_number_row_(division_number_row),
      division_number_col_(division_number_col),
      max_feature_number_(max_feature_number),
      min_feature_distance_(min_feature_distance) {}

void vslam::feature::FeatureDetectorShiTomasi::UpdateDetection(
    vslam::FeaturePositionDatabase& feature_position,
    vslam::FeatureAgeDatabase& feature_age,
    const cv::Mat& current_image) {
  database_index_t updated_max_feature_index = 0;
  DetectShiTomasiCorners(feature_position,
                         feature_age,
                         current_image,
                         division_number_row_,
                         division_number_col_,
                         max_feature_number_,
                         min_feature_distance_,
                         max_feature_index_);
}

void vslam::feature::FeatureDetectorShiTomasi::DetectShiTomasiCorners(
    vslam::FeaturePositionDatabase& feature_position,
    vslam::FeatureAgeDatabase& feature_age,
    const cv::Mat& current_image,
    int32_t division_number_row,
    int32_t division_number_col,
    int32_t max_feature_number,
    double min_feature_distance,
    vslam::database_index_t& max_feature_index) const {
  /**
   * @brief 設定項目
   * @details
   * - min_distance : goodFeaturesToTrack
   * - min_distance :
   * 新規特徴点の追加時チェック用(ほとんど同じ位置に特徴点が追加されてしまうと問題になる？)
   *   - ここむずかしい ：
   * 全く同じではないけど、ほとんど同じ位置に特徴点が追加されたとする。
   *   -
   * このとき、追加特徴点はLKで追跡した特徴点よりも正確にポイントを捉えられているはずなので、削除はしないほうが良いか？
   *   - しかし、削除しないと適切な特徴点密度が保たれなくなる。
   *   -
   * 結論：特徴点の追加操作は特徴点密度のキープが目的。特徴点位置の高精度化が目的ではないとする。特徴点位置の精度は、Verificationで確保する想定。
   *   - なので、goodFeaturesToTrackと同じ最低距離とする。
   * - division number
   * - feature point number
   */

  // Input check
  if (feature_position.size() > max_feature_number) {
    spdlog::warn("{}:{} Input feature seems to much. In:{} / Desired:{}",
                 __FILE__,
                 __FUNCTION__,
                 feature_position.size(),
                 max_feature_number);
  }

  // Generate mono channel image
  cv::Mat frame_mono;
  cv::cvtColor(current_image, frame_mono, CV_BGR2GRAY);
  auto image_size = frame_mono.size();

  // 分割しただけのGridを生成する
  auto grid_def = utility::GenerateGrid(
      image_size, division_number_col, division_number_row);
  int32_t grid_max_feature_number =
      max_feature_number / (division_number_row * division_number_col);

  // 特徴点位置マクス、KDTreeを生
  cv::Mat flag_img = cv::Mat::zeros(image_size, CV_8UC1);
  utility::pointVec kd_points;
  for (const auto& [id, pos] : feature_position) {
    if (pos[0] > 0 && pos[0] < image_size.width && pos[1] > 0 &&
        pos[1] < image_size.height) {
      flag_img.at<uint8_t>(pos[1], pos[0]) = 1;
    }
    kd_points.push_back({pos[0], pos[1]});
    //      spdlog::info("points[{}] : {}, {}", id, pos[0], pos[1]);
  }
  utility::KDTree kd_tree(kd_points);

  // gridごとに検出と、最近特徴点の削除を行う
  FeaturePositionDatabase& observing_feature_points_in_device =
      feature_position;
  FeatureAgeDatabase& feature_point_age = feature_age;

  uint64_t feature_index = max_feature_index;
  for (const auto& grect : grid_def) {
    cv::Mat div_image = frame_mono(grect);
    cv::Mat div_mask_image = flag_img(grect);

    // 事前に存在するFeature point数をカウント
    int32_t num_features = cv::countNonZero(div_mask_image);
    int32_t query_feature_number = grid_max_feature_number - num_features;
    if (query_feature_number <= 0) break;  // これ以上検出が必要なければBreak。

    // Feature pointを抽出する
    std::vector<cv::Point2f> points_from_input;
    cv::goodFeaturesToTrack(div_image,
                            points_from_input,
                            query_feature_number,
                            0.1,
                            min_feature_distance);
    std::vector<cv::KeyPoint> current_keypoints;
    cv::KeyPoint::convert(points_from_input, current_keypoints);

    // 最近傍特徴点からの距離がmin_feature_distance以上かチェック
    for (const auto& kp : current_keypoints) {
      utility::point_t check_pt{kp.pt.x, kp.pt.y};
      auto nnkp = kd_tree.nearest_point(check_pt);  // KDTreeを使ってNNPを探索。
      if (nnkp.empty()) {  // 再近傍点が発見できない場：初回など
        observing_feature_points_in_device[feature_index] =
            Vec2_t{kp.pt.x + grect.x, kp.pt.y + grect.y};
        feature_point_age[feature_index] = 1;
        feature_index++;
      } else {  // 最近傍点が発見できた場合
        if ((Vec2_t{check_pt[0], check_pt[1]} - Vec2_t{nnkp[0], nnkp[1]})
                .norm() >
            min_feature_distance) {  // 検出特徴点がNNPとmin_feature_distance以上かチェック
          observing_feature_points_in_device[feature_index] =
              Vec2_t{kp.pt.x + grect.x, kp.pt.y + grect.y};
          feature_point_age[feature_index] = 1;
          feature_index++;
        }
      }
    }
  }

  max_feature_index = feature_index - 1;
}

// vslam::data::Frame vslam::feature::FeatureDetectorShiTomasi::Detect(
//    const vslam::data::FrameSharedPtr& previous_frame,
//    const cv::Mat& current_image) {
//  database_index_t updated_max_feature_index = 0;
//  return DetectShiTomasiCorners(previous_frame,
//                                current_image,
//                                division_number_row_,
//                                division_number_col_,
//                                max_feature_number_,
//                                min_feature_distance_,
//                                max_feature_index_);
//}

// vslam::data::Frame
// vslam::feature::FeatureDetectorShiTomasi::DetectShiTomasiCorners(
//    const vslam::data::FrameSharedPtr& previous_frame,
//    const cv::Mat& current_image,
//    int32_t division_number_row,
//    int32_t division_number_col,
//    int32_t max_feature_number,
//    double min_feature_distance,
//    database_index_t& max_feature_index) const {
//  /**
//   * @brief 設定項目
//   * @details
//   * - min_distance : goodFeaturesToTrack
//   * - min_distance :
//   * 新規特徴点の追加時チェック用(ほとんど同じ位置に特徴点が追加されてしまうと問題になる？)
//   *   - ここむずかしい ：
//   * 全く同じではないけど、ほとんど同じ位置に特徴点が追加されたとする。
//   *   -
//   * このとき、追加特徴点はLKで追跡した特徴点よりも正確にポイントを捉えられているはずなので、削除はしないほうが良いか？
//   *   - しかし、削除しないと適切な特徴点密度が保たれなくなる。
//   *   -
//   * 結論：特徴点の追加操作は特徴点密度のキープが目的。特徴点位置の高精度化が目的ではないとする。特徴点位置の精度は、Verificationで確保する想定。
//   *   - なので、goodFeaturesToTrackと同じ最低距離とする。
//   * - division number
//   * - feature point number
//   */
//
//  // Generate mono channel image
//  cv::Mat frame_mono;
//  cv::cvtColor(current_image, frame_mono, CV_BGR2GRAY);
//  auto image_size = frame_mono.size();
//
//  // 分割しただけのGridを生成する
//  auto grid_def = utility::GenerateGrid(
//      image_size, division_number_col, division_number_row);
//  int32_t grid_max_feature_number =
//      max_feature_number / (division_number_row * division_number_col);
//
//  if (previous_frame) {
//    // 特徴点位置マクス、KDTreeを生
//    cv::Mat flag_img = cv::Mat::zeros(image_size, CV_8UC1);
//    utility::pointVec kd_points;
//    for (const auto& [id, pos] :
//         previous_frame->observing_feature_point_in_device_) {
//      if (pos[0] > 0 && pos[0] < image_size.width && pos[1] > 0 &&
//          pos[1] < image_size.height) {
//        flag_img.at<uint8_t>(pos[1], pos[0]) = 1;
//      }
//      kd_points.push_back({pos[0], pos[1]});
//      //      spdlog::info("points[{}] : {}, {}", id, pos[0], pos[1]);
//    }
//    utility::KDTree kd_tree(kd_points);
//
//    // gridごとに検出と、最近特徴点の削除を行う
//    std::set<database_index_t> observing_feature_ids =
//        previous_frame->observing_feature_id_;
//    //    std::set<database_index_t>
//    //    observing_feature_ids(previous_frame->observing_feature_id_);
//    EigenAllocatedUnorderedMap<database_index_t, Vec2_t>
//        observing_feature_points_in_device =
//            previous_frame->observing_feature_point_in_device_;
//    std::unordered_map<database_index_t, uint32_t> feature_point_age =
//        previous_frame->feature_point_age_;
//
//    uint64_t feature_index = max_feature_index;
//    for (const auto& grect : grid_def) {
//      cv::Mat div_image = frame_mono(grect);
//      cv::Mat div_mask_image = flag_img(grect);
//
//      // 事前に存在するFeature point数をカウント
//      int32_t num_features = cv::countNonZero(div_mask_image);
//      int32_t query_feature_number = grid_max_feature_number - num_features;
//      if (query_feature_number <= 0)
//        break;  // これ以上検出が必要なければBreak。
//
//      // Feature pointを抽出する
//      std::vector<cv::Point2f> points_from_input;
//      cv::goodFeaturesToTrack(div_image,
//                              points_from_input,
//                              query_feature_number,
//                              0.1,
//                              min_feature_distance);
//      std::vector<cv::KeyPoint> current_keypoints;
//      cv::KeyPoint::convert(points_from_input, current_keypoints);
//
//      // 最近傍特徴点からの距離がmin_feature_distance以上かチェック
//      for (const auto& kp : current_keypoints) {
//        utility::point_t check_pt{kp.pt.x, kp.pt.y};
//        auto nnkp =
//            kd_tree.nearest_point(check_pt);  // KDTreeを使ってNNPを探索。
//        if ((Vec2_t{check_pt[0], check_pt[1]} - Vec2_t{nnkp[0], nnkp[1]})
//                .norm() >
//            min_feature_distance) {  //
//            検出特徴点がNNPとmin_feature_distance以上かチェック
//          observing_feature_ids.insert(feature_index);
//          observing_feature_points_in_device[feature_index] =
//              Vec2_t{kp.pt.x + grect.x, kp.pt.y + grect.y};
//          feature_point_age[feature_index] = 1;
//          feature_index++;
//        }
//      }
//    }
//    max_feature_index = feature_index - 1;
//    return data::Frame(0,
//                       0,
//                       false,
//                       data::PinholeCameraModel(),
//                       observing_feature_ids,
//                       observing_feature_points_in_device,
//                       feature_point_age);
//
//  } else {
//    spdlog::info("Empty previous frame.");
//
//    std::set<database_index_t> observing_feature_ids;
//    EigenAllocatedUnorderedMap<database_index_t, Vec2_t>
//        observing_feature_points_in_device;
//    std::unordered_map<database_index_t, uint32_t> feature_point_age;
//
//    uint64_t feature_index = 0;
//    for (const auto& grect : grid_def) {
//      cv::Mat div_image = frame_mono(grect);
//      // Feature pointを抽出する
//      std::vector<cv::Point2f> points_from_input;
//      cv::goodFeaturesToTrack(div_image,
//                              points_from_input,
//                              grid_max_feature_number,
//                              0.1,
//                              min_feature_distance);
//      std::vector<cv::KeyPoint> current_keypoints;
//      cv::KeyPoint::convert(points_from_input, current_keypoints);
//      for (const auto& kp : current_keypoints) {
//        observing_feature_ids.insert(feature_index);
//        observing_feature_points_in_device[feature_index] =
//            Vec2_t{kp.pt.x + grect.x, kp.pt.y + grect.y};
//        feature_point_age[feature_index] = 1;
//
//        //        spdlog::info("points[{}] : {}, {}", feature_index, kp.pt.x +
//        //        grect.x, kp.pt.y + grect.y);
//        feature_index++;
//      }
//    }
//    max_feature_index = feature_index - 1;
//
//    return data::Frame(0,
//                       0,
//                       false,
//                       data::PinholeCameraModel(),
//                       observing_feature_ids,
//                       observing_feature_points_in_device,
//                       feature_point_age);
//  }
//}
