#include "dense_feature_extructor_proto.hpp"
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>

#include <omp.h>

dense_feature::dense_feature(uint64_t id_, const Eigen::Vector2i &feature_point)
    : id(id_), feature_history({feature_point}) {
  ;
}

dense_feature::dense_feature()
    : id(std::numeric_limits<uint64_t>::max()) {
}

void dense_feature::add_feature(const Eigen::Vector2i &feature) {
  feature_history.emplace_back(feature);
}
const Eigen::Vector2i dense_feature::get_latest_feature() const {
  // return *(feature_history.end());
  return feature_history[feature_history.size() - 1];
}
const std::vector<Eigen::Vector2i,
                  Eigen::aligned_allocator<Eigen::Vector2i>> &dense_feature::get_feature_history() const {
  return feature_history;
}

uint64_t dense_feature::get_id() const {
  return id;
}
void dense_feature::set_id(const uint64_t id_) {
  id = id_;
}

bool dense_feature::get_is_tracking() const {
  return is_tracking;
}
void dense_feature::set_is_tracking(const bool is_tracking_) {
  is_tracking = is_tracking_;
}

dense_feature_extructor::dense_feature_extructor() {
  is_initialize = true;
  is_initialize_dominant_flow = true;
}

/**
 * @brief feature pointクラスを使って実装し直す + Feature pointの数をキープする処理を何らか追加する
 * @details
 * 処理の流れ
 * （０．特徴点の初期化、初回のみ）
 * １．特徴点トラッキング
 * ２．特徴点の追加
 * ３．特徴点のメンテナンス
 * 
 * 初期化処理について詳しく
 * １．一様分布のランダム初期値からHill-climbして曲率ピックを行う
 * ２．重複する点があるはずなのでこれを排除
 * 
 * 特徴点の追加について詳しくて
 * これも初期化処理と同じ感じでもOKか？
 * 特徴点メンテは、重複しているものを排除する。どの特徴点を排除するのかがポイントになる。
 * 長くトラックできている特徴点を優先して残すべき？
 * 
 * こんな感じで実装してみる。
 * 
 * 
 * @param input_color 
 */
void dense_feature_extructor::detect_and_track(const cv::Mat &input_color) {
  // モノクロ画像を用意
  cv::Mat img_mono;
  cv::cvtColor(input_color, img_mono, CV_BGR2GRAY);

  // 曲率画像を生成
  cv::Mat outimg = get_curavture(img_mono);

  // dominant flowを計算
  cv::Mat dominant_affine = get_dominant_flow(img_mono);

  /**
   * @brief 初期化処理
   *
   */
  if (is_initialize) {

    features = initialize_fearure(outimg, {}, 10000);
    feature_points.reserve(features.size());

    // feature_with_frame.emplace_back(initialize_features(outimg, frame_dense_feature()));

    for (const auto &f : features) {
      feature_points.emplace_back(cv::Point2i(f.get_latest_feature()[0], f.get_latest_feature()[1]));
    }
    // features = initialize_fearure(outimg);

    // // 一様分布で特徴点初期位置
    // int32_t num_points = 10000;
    // feature_points.clear();
    // feature_points.resize(num_points);

    // std::random_device rnd;                                                          // 非決定的な乱数生成器を生成
    // std::mt19937 mt(rnd());                                                          //  メルセンヌ・ツイスタの32ビット版、引数は初期シード値
    // std::uniform_int_distribution<int32_t> rand_width(0, outimg.size().width - 1);   // [0, 99] 範囲の一様乱数
    // std::uniform_int_distribution<int32_t> rand_height(0, outimg.size().height - 1); // [0, 99] 範囲の一様乱数

    // for (size_t i = 0; i < num_points; i++)
    // {
    //     feature_points[i].x = rand_width(mt);
    //     feature_points[i].y = rand_height(mt);
    //     feature_points[i] = track_local_max(outimg, feature_points[i]);
    // }
  }

  if (!is_initialize) {

#pragma omp parallel for
    for (size_t i = 0; i < feature_points.size(); i++) {
      bool is_inside = warp_point(feature_points[i], dominant_affine, input_color.size(), feature_points[i]);
      feature_points[i] = track_local_max_with_regularization(outimg, feature_points[i]);
    }

    features = initialize_fearure(outimg, feature_points, 100);
    for (const auto &f : features) {
      feature_points.emplace_back(cv::Point2i(f.get_latest_feature()[0], f.get_latest_feature()[1]));
    }
  }

  is_initialize = false;
}

void dense_feature_extructor::detect_and_track_proto(const cv::Mat &input_color) {

  // モノクロ画像を用意
  cv::Mat img_mono;
  cv::cvtColor(input_color, img_mono, CV_BGR2GRAY);

  // 曲率画像を生成
  cv::Mat outimg = get_curavture(img_mono);

  // dominant flowを計算
  cv::Mat dominant_affine = get_dominant_flow(img_mono);

  /**
   * @brief 初期化処理
   *
   */
  if (is_initialize) {
    // 一様分布で特徴点初期位置
    int32_t num_points = 10000;
    feature_points.clear();
    feature_points.resize(num_points);

    std::random_device rnd;                                                          // 非決定的な乱数生成器を生成
    std::mt19937 mt(rnd());                                                          //  メルセンヌ・ツイスタの32ビット版、引数は初期シード値
    std::uniform_int_distribution<int32_t> rand_width(0, outimg.size().width - 1);   // [0, 99] 範囲の一様乱数
    std::uniform_int_distribution<int32_t> rand_height(0, outimg.size().height - 1); // [0, 99] 範囲の一様乱数

    for (size_t i = 0; i < num_points; i++) {
      feature_points[i].x = rand_width(mt);
      feature_points[i].y = rand_height(mt);
      feature_points[i] = track_local_max(outimg, feature_points[i]);
    }
  }

  if (!is_initialize) {
#pragma omp parallel for
    for (size_t i = 0; i < feature_points.size(); i++) {
      bool is_inside = warp_point(feature_points[i], dominant_affine, input_color.size(), feature_points[i]);
      feature_points[i] = track_local_max(outimg, feature_points[i]);
    }
  }

  is_initialize = false;
}

cv::Mat dense_feature_extructor::get_curavture(const cv::Mat &img_gray) {
  // cv::Mat img_gray;
  // cv::cvtColor(input_color, img_gray, CV_BGR2GRAY);

/**
         * @brief CV_64FC1とCV_32FC1では32bitのほうが処理が結構早い。
         * あまり結果が変わらないようなら32bitのバージョンで計算するのがよさそう。
         * 
         */
#define USE_CV_FP64
#ifdef USE_CV_FP64
  cv::Mat img_x, img_xx;
  cv::Sobel(img_gray, img_x, CV_64FC1, 1, 0, 3);
  // cv::Sobel(img_x, img_xx, CV_64FC1, 1, 0);
  cv::Sobel(img_gray, img_xx, CV_64FC1, 2, 0, 3);

  cv::Mat img_y, img_yy;
  cv::Sobel(img_gray, img_y, CV_64FC1, 0, 1, 3);
  // cv::Sobel(img_y, img_yy, CV_64FC1, 0, 1);
  cv::Sobel(img_gray, img_yy, CV_64FC1, 0, 2, 3);

  // cv::Mat img_xy;
  // cv::Sobel(img_x, img_xy, CV_64FC1, 0, 1);

  cv::Mat img_xy;
  cv::Sobel(img_gray, img_xy, CV_64FC1, 1, 1, 3);
#else
  cv::Mat img_x, img_xx;
  cv::Sobel(img_gray, img_x, CV_32FC1, 1, 0);
  cv::Sobel(img_x, img_xx, CV_32FC1, 1, 0);

  cv::Mat img_y, img_yy;
  cv::Sobel(img_gray, img_y, CV_32FC1, 0, 1);
  cv::Sobel(img_y, img_yy, CV_32FC1, 0, 1);

  cv::Mat img_xy;
  cv::Sobel(img_x, img_xy, CV_32FC1, 0, 1);
#endif

  cv::Mat img_x_p2;
  cv::multiply(img_x, img_x, img_x_p2);
  cv::Mat img_y_p2;
  cv::multiply(img_y, img_y, img_y_p2);

  cv::Mat term1, term2, term2tmp, term3;
  cv::multiply(img_xx, img_y_p2, term1);

  cv::multiply(img_x, img_y, term2tmp);
  // cv::multiply(term2, img_xy, term2, -2.0);
  cv::multiply(term2tmp, img_xy, term2);

  cv::multiply(img_yy, img_x_p2, term3);

  cv::Mat curv;
  // curv = term1 + term2 * (-2.0) + term3;
  curv = term1 + (-2.0) * term2 + term3;

  // cv::Mat outimg_normed;
  // cv::normalize(curv, outimg_normed, 0, 1, cv::NORM_MINMAX);
  // cv::imshow("Curvature", outimg_normed);

  return curv;
  // return term3;
  // return img_x;
}

cv::Point2i dense_feature_extructor::get_neighbor_max(
    const cv::Mat &img_mono,
    const cv::Point2i &input_point) {
  // std::cout << "Neigbhor ######### " << std::endl;
  // std::cout << "Neigbhor: " << input_point << std::endl;
  double max = img_mono.at<double>(input_point.y, input_point.x);
  std::vector<int32_t> dxs = {1, -1};
  std::vector<int32_t> dys = {1, -1};
  cv::Point2i max_point = input_point;

  for (const auto dx : dxs) {
    for (const auto dy : dys) {
      int32_t px, py;
      px = input_point.x + dx;
      py = input_point.y + dy;
      if ((px > 0) && (px < img_mono.size().width) && ((py > 0) && (py < img_mono.size().height))) {
        // std::cout << "Neigbhor: " << px << ", " << py << std::endl;
        auto curren_value = img_mono.at<double>(py, px);
        if (max < curren_value) {
          max = curren_value;
          max_point.x = px;
          max_point.y = py;
        }
      }
    }
  }

  return max_point;
}

cv::Point2i dense_feature_extructor::get_neighbor_max_with_regularization(
    const cv::Mat &img_mono,
    const cv::Point2i &input_point,
    const double lambda_coeff,
    const double sigma_coeff,
    const cv::Point2f &estimated_point) {
  // std::cout << "Neigbhor ######### " << std::endl;
  // std::cout << "Neigbhor: " << input_point << std::endl;
  double max = img_mono.at<double>(input_point.y, input_point.x) +
      get_regularization_term(lambda_coeff, sigma_coeff, input_point, estimated_point);
  std::vector<int32_t> dxs = {1, -1};
  std::vector<int32_t> dys = {1, -1};
  cv::Point2i max_point = input_point;

  for (const auto dx : dxs) {
    for (const auto dy : dys) {
      int32_t px, py;
      px = input_point.x + dx;
      py = input_point.y + dy;
      if ((px > 0) && (px < img_mono.size().width) && ((py > 0) && (py < img_mono.size().height))) {
        // std::cout << "Neigbhor: " << px << ", " << py << std::endl;
        auto curren_value = img_mono.at<double>(py, px) +
            get_regularization_term(lambda_coeff, sigma_coeff, cv::Point2f(px, py), estimated_point);

        if (max < curren_value) {
          max = curren_value;
          max_point.x = px;
          max_point.y = py;
        }
      }
    }
  }

  return max_point;
}

cv::Point2i dense_feature_extructor::track_local_max(
    const cv::Mat &img_mono,
    const cv::Point2i &initial_point) {
  cv::Point2i prev_neigbhor_max = initial_point;

  // std::cout << "########################" << std::endl;
  for (size_t i = 0;; i++) {
    // cv::Point2i neighbor_max = get_neighbor_max(img_mono, prev_neigbhor_max);
    cv::Point2i neighbor_max = get_neighbor_max(img_mono, prev_neigbhor_max);
    // std::cout << "Test: " << i << ", " << neighbor_max << std::endl;
    // std::cout << "PrevTest: " << i << ", " << prev_neigbhor_max << std::endl;
    if (neighbor_max == prev_neigbhor_max) {
      break;
    }

    prev_neigbhor_max = neighbor_max;
  }

  return prev_neigbhor_max;
}

cv::Point2i dense_feature_extructor::track_local_max_with_regularization(
    const cv::Mat &img_mono,
    const cv::Point2i &initial_point) {
  cv::Point2i prev_neigbhor_max = initial_point;

  // std::cout << "########################" << std::endl;
  for (size_t i = 0;; i++) {
    // cv::Point2i neighbor_max = get_neighbor_max(img_mono, prev_neigbhor_max);
    cv::Point2i neighbor_max = get_neighbor_max_with_regularization(
        img_mono, prev_neigbhor_max, 0.1, 5, initial_point);
    // std::cout << "Test: " << i << ", " << neighbor_max << std::endl;
    // std::cout << "PrevTest: " << i << ", " << prev_neigbhor_max << std::endl;
    if (neighbor_max == prev_neigbhor_max) {
      break;
    }

    prev_neigbhor_max = neighbor_max;
  }
  // cv::Point2i diff = initial_point - prev_neigbhor_max;
  // double norm = cv::norm(diff);
  // printf("Norm: %f\n", norm);

  return prev_neigbhor_max;
}

cv::Mat dense_feature_extructor::get_dominant_flow(const cv::Mat &img_mono) {

  // モノクロ化とサイズをちっさくする
  cv::Mat input_small; //, img_draw, img_mono;
  // img_color.copyTo(img_draw);
  // cv::cvtColor(img_color, img_mono, CV_BGR2GRAY);
  double scale = 1.0 / 3.0;
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
    est_affine = cv::estimateRigidTransform(keypoints_src, keypoints_dst, false);
  }

  cv::Mat match_img;
  // Feature pointを描画する
  if (is_initialize_dominant_flow) {
  } else {
    // for (const auto &p : points_from_input)
    // {
    //     cv::circle(img_draw, p * (1.0 / scale), 1, cv::Scalar(0, 0, 255), 1);
    // }
    for (const auto &good_match : good_matches) {
      auto ck = current_keypoints[good_match.queryIdx].pt * (1.0 / scale);
      auto pk = prev_keypoints[good_match.trainIdx].pt * (1.0 / scale);
      cv::Mat pk_mat = (cv::Mat_<double>(3, 1) << pk.x, pk.y, 1);

      cv::Mat ck_est = est_affine * pk_mat;
      cv::Point2f ck_est_pt(ck_est.at<double>(0, 0), ck_est.at<double>(1, 0));

      // auto ck = current_keypoints[good_match.trainIdx].pt * (1.0 / scale);
      // auto pk = prev_keypoints[good_match.queryIdx].pt * (1.0 / scale);

      // cv::circle(img_draw, ck, 2, cv::Scalar(255, 0, 0), 1, CV_AA);
      // cv::circle(img_draw, pk, 2, cv::Scalar(0, 255, 0), 1, CV_AA);
      // cv::line(img_draw, ck, pk, cv::Scalar(255, 255, 255), 1, CV_AA);
      // cv::line(img_draw, pk, ck_est_pt, cv::Scalar(255, 0, 255), 1, CV_AA);
      // cv::circle(img_draw, ck_est_pt, 2, cv::Scalar(0, 0, 255), 1, CV_AA);
    }

    // cv::drawMatches(input_small, current_keypoints, prev_img, prev_keypoints, good_matches, match_img);
    // cv::imshow("match", match_img);
  }

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

double dense_feature_extructor::get_regularization_term(
    const double lambda_coeff,
    const double sigma_coeff,
    const cv::Point2f &input_point,
    const cv::Point2f &estimated_point) {
  double dest = cv::norm(input_point - estimated_point);
  double rho = dest * dest / (dest * dest + sigma_coeff);
  double omega = 1 - rho;
  double reg = lambda_coeff * omega;

  return reg;
}

std::vector<cv::Point2f> dense_feature_extructor::warp_points(
    const std::vector<cv::Point2f> &inputs,
    cv::Mat &affine_mat,
    cv::Size image_size) {
  std::vector<cv::Point2f> outputs;
  outputs.reserve(inputs.size());

  for (const auto input : inputs) {
    cv::Mat input_mat = (cv::Mat_<double>(3, 1) << input.x, input.y, 1);

    cv::Mat ck_est = affine_mat * input_mat;
    cv::Point2f ck_est_pt(ck_est.at<double>(0, 0), ck_est.at<double>(1, 0));

    if ((ck_est_pt.x >= 0) && (ck_est_pt.x < image_size.width) && (ck_est_pt.y >= 0)
        && (ck_est_pt.y < image_size.height)) {
      outputs.emplace_back(ck_est_pt);
    }
  }

  return outputs;
}

bool dense_feature_extructor::warp_point(
    const cv::Point2f &input,
    const cv::Mat &affine_mat,
    const cv::Size &image_size,
    cv::Point2f &output) {
  cv::Mat input_mat = (cv::Mat_<double>(3, 1) << input.x, input.y, 1);
  cv::Mat ck_est = affine_mat * input_mat;
  cv::Point2f ck_est_pt(ck_est.at<double>(0, 0), ck_est.at<double>(1, 0));

  if ((ck_est_pt.x >= 0) && (ck_est_pt.x < image_size.width) && (ck_est_pt.y >= 0)
      && (ck_est_pt.y < image_size.height)) {
    output = ck_est_pt;
    return true;
  } else {
    output = input;
    return false;
  }
}

const std::vector<cv::Point2f> &dense_feature_extructor::get_feature_points() const {
  return feature_points;
}

std::vector<dense_feature> dense_feature_extructor::initialize_fearure(
    const cv::Mat &img_curvature,
    const std::vector<cv::Point2f> &pre_points,
    const int32_t num_points) {
  std::vector<dense_feature> feature_points;
  feature_points.reserve(num_points);
  std::random_device rnd;                                                                 // 非決定的な乱数生成器を生成
  std::mt19937
      mt(rnd());                                                                 //  メルセンヌ・ツイスタの32ビット版、引数は初期シード値
  std::uniform_int_distribution<int32_t> rand_width(0, img_curvature.size().width - 1);   // [0, 99] 範囲の一様乱数
  std::uniform_int_distribution<int32_t> rand_height(0, img_curvature.size().height - 1); // [0, 99] 範囲の一様乱数

  int32_t del_count = 0;

  cv::Mat flag_img(img_curvature.size(), CV_8U);
  flag_img = 0;

  // // prepointsをフラグに書き込む
  // for (const auto &p : pre_points)
  // {
  //     if (p.x > 0 && p.x < img_curvature.size().width && p.y > 0 && p.y < img_curvature.size().width)
  //     {
  //         std::cout << p << std::endl;
  //         flag_img.at<uint8_t>(static_cast<int32_t>(p.y), static_cast<int32_t>(p.x)) = 1;
  //     }
  // }
  // for (size_t i = 0; i < pre_points.size(); i++)
  // {
  //     flag_img.at<uint8_t>((pre_points[i].y), (pre_points[i].x)) = 1;
  // }

  for (size_t i = 0; i < num_points; i++) {
    cv::Point2i tmp_point(rand_width(mt), rand_height(mt));
    tmp_point = track_local_max(img_curvature, tmp_point);

    uint8_t flag = flag_img.at<uint8_t>(tmp_point.y, tmp_point.x);
    double curv = img_curvature.at<uint8_t>(tmp_point.y, tmp_point.x);
    if ((!flag) && (curv > 200.0)) {
      feature_points.emplace_back(dense_feature(0, Eigen::Vector2i(tmp_point.x, tmp_point.y)));
      flag_img.at<uint8_t>(tmp_point.y, tmp_point.x) = 1;
    } else {
      del_count++;
    }
  }

  return feature_points;
}

/**
     * @brief フレーム中の特徴点を初期化する
     * 
     * @param img_curvature 
     * @param prev_features 
     * @param num_points 
     * @return frame_dense_feature 
     */
frame_dense_feature dense_feature_extructor::initialize_features(
    const cv::Mat &img_curvature,
    const frame_dense_feature &prev_features,
    const int32_t num_points) {

  frame_dense_feature features;
  features.fearture_points = prev_features.fearture_points;

  std::random_device rnd;                                                                 // 非決定的な乱数生成器を生成
  std::mt19937
      mt(rnd());                                                                 //  メルセンヌ・ツイスタの32ビット版、引数は初期シード値
  std::uniform_int_distribution<int32_t> rand_width(0, img_curvature.size().width - 1);   // [0, 99] 範囲の一様乱数
  std::uniform_int_distribution<int32_t> rand_height(0, img_curvature.size().height - 1); // [0, 99] 範囲の一様乱数

  int32_t del_count = 0;

  // cv::Mat flag_img(img_curvature.size(), CV_8U);
  cv::Mat flag_img = cv::Mat::zeros(img_curvature.size(), CV_8U);
  // cv::Mat flag_img(cv::Size(640, 480), CV_8U);
  flag_img = 0;

  uint64_t max_id = 0;
  // for (const auto &[id, point] : prev_features.fearture_points)
  // {
  //     if (max_id < id)
  //     {
  //         max_id = id;
  //     }
  //     printf("## %ld,  %f, %f\n", id, point[0], point[1]);
  //     flag_img.at<uint8_t>(point[1], point[0]) = 1;
  //     // flag_img.at<uint8_t>(0, 0) = 1;
  // }

  for (auto i = features.fearture_points.begin(); i != features.fearture_points.end(); ++i) {
    uint64_t id = i->first;
    Eigen::Vector2i p = i->second;
    if (max_id < id) {
      max_id = id;
    }
    // printf("## %ld,  %f, %f\n", id, p[0], p[1]);
    // flag_img.at<uint8_t>(p[1], p[0]) = 1;
  }

  for (size_t i = 0; i < num_points; i++) {
    cv::Point2i tmp_point(rand_width(mt), rand_height(mt));
    tmp_point = track_local_max(img_curvature, tmp_point);

    uint8_t flag = flag_img.at<uint8_t>(tmp_point.y, tmp_point.x);
    double curv = img_curvature.at<uint8_t>(tmp_point.y, tmp_point.x);
    if ((!flag)) {
      features.fearture_points[max_id] = Eigen::Vector2i(tmp_point.x, tmp_point.y);
      max_id++;
      flag_img.at<uint8_t>(tmp_point.y, tmp_point.x) = 1;
    } else {
      del_count++;
    }
  }

  return features;
}