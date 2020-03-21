#include "dense_feature_extructor.hpp"
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <omp.h>

using namespace dense_feature;

dense_feature_extructor::dense_feature_extructor(const double lambda_,
                                                 const double sigma_,
                                                 const double dominant_flow_scale_)
    : lambda(lambda_), sigma(sigma_), dominant_flow_scale(dominant_flow_scale_) {
  is_initialize = true;
  is_initialize_dominant_flow = true;
}

void dense_feature_extructor::detect_and_track(const cv::Mat &input_color, bool is_color) {
  // モノクロ画像を用意
  cv::Mat img_mono;
  if (is_color) {
    cv::cvtColor(input_color, img_mono, CV_BGR2GRAY);
  } else {
    input_color.copyTo(img_mono);
  }

  // 曲率画像を生成
  cv::Mat outimg = utils::generate_curvature_image(img_mono);

  // dominant flowを計算
  cv::Mat dominant_affine = get_dominant_flow(img_mono);

  /**
   * @brief 初期化処理
   *
   */
  if (is_initialize) {

    features.emplace_back(initialize_features(outimg, feature_in_frame(), 10000));
  }

  if (!is_initialize) {
    auto &prev_feature = (features[features.size() - 1]); // 前回の特徴点
    feature_in_frame current_feature;

    // #pragma omp parallel for
    for (size_t i = 0; i < prev_feature.features.size(); i++) {
      cv::Point2f prev_point(prev_feature.features[i][0], prev_feature.features[i][1]);

      bool is_inside = utils::warp_point(prev_point, dominant_affine, input_color.size(), prev_point);

      if (is_inside) {
        cv::Point2f prev_point_mod = utils::track_local_max_with_regularization(outimg, prev_point);
        double mod_diff = cv::norm(prev_point_mod - prev_point);
        if (mod_diff > 0.0) {
          current_feature.featureIDs.emplace_back(prev_feature.featureIDs[i]);
          current_feature.features.emplace_back(Eigen::Vector2i(prev_point_mod.x, prev_point_mod.y));
        }
      }
    }

    features.emplace_back(initialize_features(outimg, current_feature, 10, 10, 3000));
//      features.emplace_back(initialize_features(outimg, current_feature, 10, 10, 1000));
//         features.emplace_back(initialize_features(outimg, current_feature, 10, 10, 300));
  }

  is_initialize = false;
}

cv::Mat dense_feature_extructor::get_dominant_flow(const cv::Mat &img_mono) {
  // モノクロ化とサイズをちっさくする
  cv::Mat input_small; //, img_draw, img_mono;
  // img_color.copyTo(img_draw);
  // cv::cvtColor(img_color, img_mono, CV_BGR2GRAY);
  // double scale = 1.0 / 3.0;
  double scale = dominant_flow_scale;
  cv::resize(img_mono, input_small, cv::Size(), scale, scale, CV_INTER_LINEAR);

  // Feature pointを抽出する
  std::vector<cv::Point2f> points_from_input;
  cv::goodFeaturesToTrack(input_small, points_from_input, 1000, 0.01, 0.1);
  std::vector<cv::KeyPoint> current_keypoints;
  cv::KeyPoint::convert(points_from_input, current_keypoints);

  // 特徴量を計算する
  auto feature = cv::xfeatures2d::BriefDescriptorExtractor::create(32);
  cv::Mat current_descriptor;
  feature->compute(input_small, current_keypoints, current_descriptor);

  // feature matching、最初のフレームは計算しない
  std::vector<cv::DMatch> good_matches;
  cv::Mat est_affine;
  if (!is_initialize_dominant_flow) {
    // auto matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::BRUTEFORCE_HAMMING);
    auto matcher = cv::BFMatcher::create(cv::NORM_HAMMING);
    std::vector<std::vector<cv::DMatch>> knn_matches;
    matcher->knnMatch(current_descriptor, prev_descriptor, knn_matches, 2);

    //-- Filter matches using the Lowe's ratio test
    const float ratio_thresh = 0.5f;
    for (size_t i = 0; i < knn_matches.size(); i++) {
      if (knn_matches[i][0].distance < ratio_thresh * knn_matches[i][1].distance) {
        good_matches.push_back(knn_matches[i][0]);
      }
    }

    // Affineの推定を行う
    std::vector<cv::Point2f> keypoints_src, keypoints_dst;
    for (const auto &good_match : good_matches) {
      auto ck = current_keypoints[good_match.queryIdx].pt * (1.0 / scale);
      auto pk = prev_keypoints[good_match.trainIdx].pt * (1.0 / scale);
      keypoints_src.emplace_back(pk);
      keypoints_dst.emplace_back(ck);
    }

    if (keypoints_src.size() > 4) {
      est_affine = cv::estimateRigidTransform(keypoints_src, keypoints_dst, false);
    }

    if (est_affine.size() == cv::Size(0, 0)) {
      est_affine = (cv::Mat_<double>(2, 3) << 1, 0, 0, 0, 1, 0);
    }

    // printf("size: %ld, %ld\n", keypoints_src.size(), keypoints_dst.size());
    // std::cout << est_affine << std::endl;
  }

  // cv::Mat match_img;
  // // Feature pointを描画する
  // if (is_initialize_dominant_flow)
  // {
  // }
  // else
  // {
  //     // for (const auto &p : points_from_input)
  //     // {
  //     //     cv::circle(img_draw, p * (1.0 / scale), 1, cv::Scalar(0, 0, 255), 1);
  //     // }
  //     for (const auto &good_match : good_matches)
  //     {
  //         auto ck = current_keypoints[good_match.queryIdx].pt * (1.0 / scale);
  //         auto pk = prev_keypoints[good_match.trainIdx].pt * (1.0 / scale);
  //         cv::Mat pk_mat = (cv::Mat_<double>(3, 1) << pk.x, pk.y, 1);

  //         cv::Mat ck_est = est_affine * pk_mat;
  //         cv::Point2f ck_est_pt(ck_est.at<double>(0, 0), ck_est.at<double>(1, 0));

  //         // auto ck = current_keypoints[good_match.trainIdx].pt * (1.0 / scale);
  //         // auto pk = prev_keypoints[good_match.queryIdx].pt * (1.0 / scale);

  //         // cv::circle(img_draw, ck, 2, cv::Scalar(255, 0, 0), 1, CV_AA);
  //         // cv::circle(img_draw, pk, 2, cv::Scalar(0, 255, 0), 1, CV_AA);
  //         // cv::line(img_draw, ck, pk, cv::Scalar(255, 255, 255), 1, CV_AA);
  //         // cv::line(img_draw, pk, ck_est_pt, cv::Scalar(255, 0, 255), 1, CV_AA);
  //         // cv::circle(img_draw, ck_est_pt, 2, cv::Scalar(0, 0, 255), 1, CV_AA);
  //     }

  //     // cv::drawMatches(input_small, current_keypoints, prev_img, prev_keypoints, good_matches, match_img);
  //     // cv::imshow("match", match_img);
  // }

  // cv::imshow("feature", img_draw);

  // 後処理
  if (is_initialize_dominant_flow) {
    is_initialize_dominant_flow = false;
  }
  prev_descriptor = current_descriptor;
  prev_keypoints = current_keypoints;
  input_small.copyTo(prev_img);

  return est_affine;
}

feature_in_frame dense_feature_extructor::initialize_features(
    const cv::Mat &img_curvature,
    const feature_in_frame &previous_frame,
    const int32_t num_grids_x,
    const int32_t num_grids_y,
    const int32_t num_points) {
  feature_in_frame current_features = previous_frame;

  // 乱数初期化
  std::random_device rnd; // 非決定的な乱数生成器を生成
  std::mt19937 mt(rnd()); //  メルセンヌ・ツイスタの32ビット版、引数は初期シード値
  // std::uniform_int_distribution<int32_t> rand_width(0, img_curvature.size().width - 1);   // [0, 99] 範囲の一様乱数
  // std::uniform_int_distribution<int32_t> rand_height(0, img_curvature.size().height - 1); // [0, 99] 範囲の一様乱数

  // 検出済みマスク
  cv::Mat flag_img = cv::Mat::zeros(img_curvature.size(), CV_8UC1);
  cv::Mat_<uchar> flag_img_ac = cv::Mat_<uchar>(flag_img);
  flag_img = 0;

  // 画像をグリッドで分割して均等な分布になるようにする
  double dx, dy;
  dx = static_cast<double>(img_curvature.size().width) / static_cast<double>(num_grids_x);
  dy = static_cast<double>(img_curvature.size().height) / static_cast<double>(num_grids_y);
  std::vector<cv::Rect2f> grid_def(0);
  grid_def.reserve(num_grids_x * num_grids_y);
  for (int32_t x = 0; x < num_grids_x; x++) {
    for (int32_t y = 0; y < num_grids_y; y++) {
      cv::Point2f left_up(x * dx, y * dy);
      cv::Point2f right_down = left_up + cv::Point2f(dx, dy);
      grid_def.emplace_back(cv::Rect2f(left_up, right_down));
    }
  }

  // 事前に検出されている特徴点を登録する
  for (const auto &p : current_features.features) {
    if (p[0] > 0 && p[0] < img_curvature.size().width && p[1] > 0 && p[1] < img_curvature.size().height) {
      flag_img_ac(p[1], p[0]) = 1;
    }
  }

  // 現状のMAXID
  uint64_t max_id = 0;
  if (!(current_features.featureIDs.size() == 0)) {
    max_id = *(std::max_element(current_features.featureIDs.begin(), current_features.featureIDs.end()));
  }
  // printf("max id : %ld\n", max_id);

  // Gridに分割して特徴点の追加を行う
  int32_t num_max_features = num_points / (num_grids_x * num_grids_y);

  // cv::Mat mask;
  // utils::non_maxima_suppression(img_curvature, mask, true);
  // std::vector<cv::Point2i> locations; // output, locations of non-zero pixels
  // cv::findNonZero(mask, locations);

  for (const auto &grect : grid_def) {
    int32_t num_features = cv::countNonZero(flag_img(grect)); // すでに存在するGrid内の特徴点数をカウントする

    std::uniform_int_distribution<int32_t> grid_rand_width(grect.x, grect.width + grect.x - 1);
    std::uniform_int_distribution<int32_t> grid_rand_height(grect.y, grect.height + grect.y - 1);
    for (size_t i = num_features; i < num_max_features; i++) {
      cv::Point2i tmp_point(grid_rand_width(mt), grid_rand_height(mt));
      tmp_point = utils::track_local_max(img_curvature, tmp_point);

      uint8_t flag = flag_img.at<uint8_t>(tmp_point.y, tmp_point.x);
      double curv = img_curvature.at<uint8_t>(tmp_point.y, tmp_point.x);
      if ((!flag)) {
        max_id++;
        current_features.features.emplace_back(Eigen::Vector2i(tmp_point.x, tmp_point.y));
        current_features.featureIDs.emplace_back(max_id);
        flag_img_ac(tmp_point.y, tmp_point.x) = 1;
      }
    }

    // for (size_t i = 0; i < locations.size(); i++)
    // {
    //     cv::Point2i tmp_point = locations[i];
    //     if (grect.contains(tmp_point))
    //     {
    //         tmp_point = utils::track_local_max(img_curvature, tmp_point);
    //         uint8_t flag = flag_img.at<uint8_t>(tmp_point.y, tmp_point.x);
    //         double curv = img_curvature.at<uint8_t>(tmp_point.y, tmp_point.x);
    //         if ((!flag))
    //         {
    //             max_id++;
    //             current_features.features.emplace_back(Eigen::Vector2i(tmp_point.x, tmp_point.y));
    //             current_features.featureIDs.emplace_back(max_id);
    //             flag_img_ac(tmp_point.y, tmp_point.x) = 1;

    //             num_features++;
    //             if (num_features > num_max_features)
    //             {
    //                 break;
    //             }
    //         }
    //     }
    // }
  }

  // 特徴点を検出、追加する // 全体対象の追加
  // for (size_t i = 0; i < num_points; i++)
  // {
  //     cv::Point2i tmp_point(rand_width(mt), rand_height(mt));
  //     tmp_point = utils::track_local_max(img_curvature, tmp_point);

  //     uint8_t flag = flag_img.at<uint8_t>(tmp_point.y, tmp_point.x);
  //     double curv = img_curvature.at<uint8_t>(tmp_point.y, tmp_point.x);
  //     // if ((!flag) && (curv > 200))
  //     if ((!flag))
  //     {
  //         max_id++;
  //         current_features.features.emplace_back(Eigen::Vector2i(tmp_point.x, tmp_point.y));
  //         current_features.featureIDs.emplace_back(max_id);
  //         // flag_img.at<uint8_t>(tmp_point.y, tmp_point.x) = 1;

  //         flag_img_ac(tmp_point.y, tmp_point.x) = 1;
  //     }
  // }

  return current_features;
}
