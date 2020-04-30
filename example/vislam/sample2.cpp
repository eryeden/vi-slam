#include <iostream>
#include <opencv2/opencv.hpp>
// #include "dense_feature_extructor_proto.hpp"
#include "dense_feature_extructor.hpp"

#include "log_util.h"

cv::Scalar HSVtoRGB(double H, double S, double V) {
  double C = S * V;
  double X = C * (1 - abs(fmod(H / 60.0, 2) - 1));
  double m = V - C;
  double Rs, Gs, Bs;

  if (H >= 0 && H < 60) {
    Rs = C;
    Gs = X;
    Bs = 0;
  } else if (H >= 60 && H < 120) {
    Rs = X;
    Gs = C;
    Bs = 0;
  } else if (H >= 120 && H < 180) {
    Rs = 0;
    Gs = C;
    Bs = X;
  } else if (H >= 180 && H < 240) {
    Rs = 0;
    Gs = X;
    Bs = C;
  } else if (H >= 240 && H < 300) {
    Rs = X;
    Gs = 0;
    Bs = C;
  } else {
    Rs = C;
    Gs = 0;
    Bs = X;
  }

  return cv::Scalar((Rs + m) * 255, (Gs + m) * 255, (Bs + m) * 255);
}

int main() {

  /**
   * @brief 描画用の色を生成する
   *
   */
  uint32_t num_colors = 1000;
  std::vector<cv::Scalar> colors(num_colors);
  for (size_t i = 0; i < colors.size(); i++) {
    double h, s, v;
    h = static_cast<double>(i) / static_cast<double>(num_colors) * 360.0;
    colors[i] = HSVtoRGB(h, 1, 1);
  }

  // LogPlayer_extended lpe("/home/ery/Devel/tmp/assets/20191219_2/20191219_24", 0);

  // dense_feature_extructor dfe;
  dense_feature::dense_feature_extructor dfe(0.1, 0.1);

  // dfe.run_extruction("/home/ery/assets/20191219_2/20191219_31");
  // dfe.run_extruction("/home/ery/assets/20191219_1/20191219_2");
  // dfe.run_extruction("/home/ery/assets/20191115/20191115_40_2m_track");

  // dfe.run_extruction("/home/ery/Devel/tmp/assets/20191219_2/20191219_31");
  // dfe.run_extruction_cam("/dev/video0", 1.0);
  // dfe.run_extruction_cam("/home/ery/Devel/tmp/assets/IMG_5144.MOV", 1.0 / 2.0);

  // std::string path_to_log_dir = "/home/ery/Devel/tmp/assets/20191115/20191115_40_2m_track";
  std::string path_to_log_dir = "/home/ery/Devel/tmp/assets/20191219_1/20191219_3";
  // std::string path_to_log_dir = "/home/ery/Devel/tmp/assets/20191219_2/20191219_31";
  LogPlayer_extended lpe(path_to_log_dir, 0.01);

  // LogPlayer_euroc_mav lp_mav("/home/ery/Downloads/V1_01_easy/mav0/cam0", 0.001);

  int64_t ref_size = 5;

  for (size_t i = 0; i < lpe.get_frame_size(); i++) {
    cv::Mat img, img_undistort;
    double tstamp;
    lpe.get_frame_by_index(img, tstamp, i);

    // カメラ画像を補正するようにする
    // カメラの歪み補正 パラメータ FIXME 外用Econカメラの4:3画像サイズの補正用パラメータなので、カメラでパラメータを変更できるようにしなければならない
    cv::Mat intrinsic_matrix(3, 3, CV_32FC1);
    intrinsic_matrix = (cv::Mat_<float>(3, 3) << 2.9055658344721849e+02, 0.0000000000000000e+00, 3.3084971542082224e+02,
        0.0000000000000000e+00, 2.9090444676137702e+02, 2.3369200839351839e+02,
        0.0000000000000000e+00, 0.0000000000000000e+00, 1.0000000000000000e+00);
    cv::Mat distortion_coeffs(5, 1, CV_32FC1);
    distortion_coeffs = (cv::Mat_<float>(5, 1)
        << -2.4556825095656906e-01, 7.5388587025469550e-02, 1.3153851332293872e-03, -1.3332173710016491e-04, -1.0879007108602111e-02);
    cv::undistort(img, img_undistort, intrinsic_matrix, distortion_coeffs);

    dfe.detect_and_track(img_undistort);

    // for (size_t fnum = std::max(static_cast<int64_t>(dfe.features.size()) - ref_size, 0l);
    //      fnum < dfe.features.size(); fnum++)
    // {
    //     for (size_t i = 0; i < dfe.features[fnum].features.size(); i++)
    //     {
    //         auto &f = dfe.features[fnum];
    //         cv::circle(img_undistort, cv::Point2i(f.features[i][0], f.features[i][1]), 1, colors[f.featureIDs[i] % num_colors], 1);
    //     }
    // }

    std::map<uint64_t, std::vector<cv::Point2i>> feature_lists;
    for (size_t i = 0; i < (dfe.features[dfe.features.size() - 1]).features.size(); i++) {
      auto &f = dfe.features[dfe.features.size() - 1];
      feature_lists[f.featureIDs[i]] = std::vector<cv::Point2i>({cv::Point2i(f.features[i][0], f.features[i][1])});
    }

    for (size_t fnum = dfe.features.size() - 1;
         fnum > std::max(static_cast<int64_t>(dfe.features.size()) - ref_size, 0l); fnum--) {
      for (size_t i = 0; i < dfe.features[fnum].features.size(); i++) {
        auto &f = dfe.features[fnum];
        if (feature_lists.count(f.featureIDs[i]) != 0) {
          feature_lists[f.featureIDs[i]].emplace_back(cv::Point2i(f.features[i][0], f.features[i][1]));
        }
      }
    }

    double maxlen = 0;
    for (const auto &[id, p] : feature_lists) {
      cv::Point2i d = p[0] - p[p.size() - 1];
      if (maxlen < cv::norm(d)) {
        maxlen = cv::norm(d);
      }
    }

    for (const auto &[id, p] : feature_lists) {
      cv::Point2i d = p[0] - p[p.size() - 1];
      double angle = std::atan2(d.y, d.x) * 180.0 / M_PI;
      double len = cv::norm(d);
      angle += 180;

      // cv::Scalar dcolor = HSVtoRGB(angle, 1, 1);
      // cv::Scalar dcolor = HSVtoRGB(len / maxlen * 360.0, 1, 1);
      cv::Scalar dcolor = colors[id % num_colors];

      // cv::polylines(img_color, p, false, colors[id % num_colors]);
      // cv::polylines(img_undistort, p, false, dcolor, 1);
      cv::circle(img_undistort, p[0], 2, dcolor, 1);
    }

    // for (const auto &[id, p] : feature_lists)
    // {
    //     cv::Point2i d = p[0] - p[p.size() - 1];
    //     double angle = std::atan2(d.y, d.x) * 180.0 / M_PI;
    //     angle += 180;
    //     // cv::polylines(img_color, p, false, colors[id % num_colors]);
    //     cv::polylines(img_undistort, p, false, HSVtoRGB(angle, 1, 1), 1);
    //     cv::circle(img_undistort, p[0], 2, HSVtoRGB(angle, 1, 1), 1);
    // }

    // for (size_t i = 0; i < dfe.features[dfe.features.size() - 1].features.size(); i++)
    // {
    //     auto &f = dfe.features[dfe.features.size() - 1];
    //     cv::circle(img_undistort, cv::Point2i(f.features[i][0], f.features[i][1]), 1, colors[f.featureIDs[i] % num_colors], 1);
    // }

    cv::imshow("feature", img_undistort);
    cv::waitKey(1);
  }
}