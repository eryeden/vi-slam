#include <iostream>
#include <opencv2/opencv.hpp>
#include "dense_feature_extructor.hpp"

#include "log_util.h"

int main()
{

    // LogPlayer_extended lpe("/home/ery/Devel/tmp/assets/20191219_2/20191219_24", 0);

    dense_feature_extructor dfe;
    // dfe.run_extruction("/home/ery/assets/20191219_2/20191219_31");
    // dfe.run_extruction("/home/ery/assets/20191219_1/20191219_2");
    // dfe.run_extruction("/home/ery/assets/20191115/20191115_40_2m_track");

    // dfe.run_extruction("/home/ery/Devel/tmp/assets/20191219_2/20191219_31");
    // dfe.run_extruction_cam("/dev/video0", 1.0);
    // dfe.run_extruction_cam("/home/ery/Devel/tmp/assets/IMG_5144.MOV", 1.0 / 2.0);

    // std::string path_to_log_dir = "/home/ery/assets/20191115/20191115_40_2m_track";
    // std::string path_to_log_dir = "/home/ery/Devel/tmp/assets/20191219_1/20191219_3";
    std::string path_to_log_dir = "/home/ery/Devel/tmp/assets/20191219_2/20191219_31";
    LogPlayer_extended lpe(path_to_log_dir, 0.01);

    for (size_t i = 0; i < lpe.get_frame_size(); i++)
    {
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
        distortion_coeffs = (cv::Mat_<float>(5, 1) << -2.4556825095656906e-01, 7.5388587025469550e-02, 1.3153851332293872e-03, -1.3332173710016491e-04, -1.0879007108602111e-02);
        cv::undistort(img, img_undistort, intrinsic_matrix, distortion_coeffs);

        dfe.detect_and_track(img_undistort);

        std::vector<cv::Point2f> feature_points = dfe.get_feature_points();

        for (const auto p : feature_points)
        {
            cv::circle(img_undistort, p, 1, cv::Scalar(255, 0, 0), 1);
        }

        cv::imshow("feature", img_undistort);
        cv::waitKey(10);
    }

    // cv::VideoCapture cap("/dev/video0");

    // for (size_t i = 0;; i++)
    // {
    //     cv::Mat img, img_undistort;
    //     double tstamp;
    //     // lpe.get_frame_by_index(img, tstamp, i);
    //     cap >> img;
    //     dfe.detect_and_track(img);

    //     std::vector<cv::Point2f> feature_points = dfe.get_feature_points();

    //     for (const auto p : feature_points)
    //     {
    //         cv::circle(img, p, 1, cv::Scalar(255, 0, 0), 1);
    //     }

    //     cv::imshow("feature", img);
    //     cv::waitKey(1);
    // }
}