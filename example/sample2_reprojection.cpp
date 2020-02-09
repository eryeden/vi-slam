#include <iostream>
#include <algorithm>

#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/viz.hpp>

#include <Eigen/Core>
#include <opencv2/core/eigen.hpp>

#include "dense_feature_extructor.hpp"
#include "initializer.hpp"

#include "log_util.h"

cv::Scalar HSVtoRGB(double H, double S, double V)
{
    double C = S * V;
    double X = C * (1 - abs(fmod(H / 60.0, 2) - 1));
    double m = V - C;
    double Rs, Gs, Bs;

    if (H >= 0 && H < 60)
    {
        Rs = C;
        Gs = X;
        Bs = 0;
    }
    else if (H >= 60 && H < 120)
    {
        Rs = X;
        Gs = C;
        Bs = 0;
    }
    else if (H >= 120 && H < 180)
    {
        Rs = 0;
        Gs = C;
        Bs = X;
    }
    else if (H >= 180 && H < 240)
    {
        Rs = 0;
        Gs = X;
        Bs = C;
    }
    else if (H >= 240 && H < 300)
    {
        Rs = X;
        Gs = 0;
        Bs = C;
    }
    else
    {
        Rs = C;
        Gs = 0;
        Bs = X;
    }

    return cv::Scalar((Rs + m) * 255, (Gs + m) * 255, (Bs + m) * 255);
}

Eigen::Vector2d reproject_point(const Eigen::Matrix3d &cam_rotation_mat,
                                const Eigen::Vector3d &cam_translation_vec,
                                const Eigen::Matrix3d &cam_intrinsic_mat,
                                const Eigen::Vector3d &point_in_3d)
{
    Eigen::Vector4d point_in_3d_tmp(point_in_3d[0], point_in_3d[1], point_in_3d[2], 1);
    Eigen::Vector3d point_in_device_tmp;
    Eigen::Vector2d point_in_device_frame;
    Eigen::Matrix<double, 3, 4> projection_mat, tmp_translation_mat;
    tmp_translation_mat << Eigen::MatrixXd::Identity(3, 3), -cam_translation_vec;
    // std::cout << "[" << tmp_translation_mat << "]" << std::endl;
    projection_mat = cam_intrinsic_mat * cam_rotation_mat.transpose() * tmp_translation_mat;
    // std::cout << "[" << projection_mat << "]" << std::endl;
    point_in_device_tmp = projection_mat * point_in_3d_tmp;
    point_in_device_frame = Eigen::Vector2d(point_in_device_tmp[0] / point_in_device_tmp[2], point_in_device_tmp[1] / point_in_device_tmp[2]);
    // std::cout << "[" << point_in_device_frame << "]" << std::endl;
    return point_in_device_frame;
}

int main()
{

    /**
     * @brief 描画用の色を生成する
     * 
     */
    uint32_t num_colors = 1000;
    std::vector<cv::Scalar> colors(num_colors);
    for (size_t i = 0; i < colors.size(); i++)
    {
        double h, s, v;
        h = static_cast<double>(i) / static_cast<double>(num_colors) * 360.0;
        colors[i] = HSVtoRGB(h, 1, 1);
    }

    dense_feature::dense_feature_extructor dfe(0.1, 0.1);

    //LogPlayer_euroc_mav lp_mav("/home/ery/Downloads/V1_01_easy/mav0/cam0", 0.001);
    LogPlayer_euroc_mav lp_mav("/home/ery/assets/V1_01_easy/mav0/cam0", 0.001);
    // LogPlayer_euroc_mav lp_mav("/home/ery/Downloads/V2_01_easy/mav0/cam0", 0.001);
    // LogPlayer_euroc_mav lp_mav("/e/subspace/tmp/tmp/V1_01_easy/mav0/cam0", 0.001);
    // LogPlayer_euroc_mav lp_mav("/e/subspace/tmp/tmp/MH_01_easy/mav0/cam0", 0.001);

    // // カメラ画像を補正するようにする
    // // カメラの歪み補正 パラメータ FIXME 外用Econカメラの4:3画像サイズの補正用パラメータなので、カメラでパラメータを変更できるようにしなければならない
    cv::Mat intrinsic_matrix(3, 3, CV_32FC1);
    intrinsic_matrix = (cv::Mat_<float>(3, 3) << 458.654, 0.0000000000000000e+00, 367.215,
                        0.0000000000000000e+00, 457.296, 248.375,
                        0.0000000000000000e+00, 0.0000000000000000e+00, 1.0000000000000000e+00);
    cv::Mat intrinsic_matrix_d(3, 3, CV_64FC1);
    intrinsic_matrix_d = (cv::Mat_<double>(3, 3) << 458.654, 0.0000000000000000e+00, 367.215,
                          0.0000000000000000e+00, 457.296, 248.375,
                          0.0000000000000000e+00, 0.0000000000000000e+00, 1.0000000000000000e+00);
    Eigen::Matrix3d intrinsic_eigen;
    cv::cv2eigen(intrinsic_matrix_d, intrinsic_eigen);

    cv::Mat distortion_coeffs(5, 1, CV_32FC1);
    distortion_coeffs = (cv::Mat_<float>(5, 1) << -0.28340811, 0.07395907, 0.00019359, 1.76187114e-05);

    cv::Vec3f cam_pos(3.0f, 3.0f, 3.0f), cam_focal_point(3.0f, 3.0f, 2.0f), cam_y_dir(-1.0f, 0.0f, 0.0f);

    // cv::Affine3f cam_pose = cv::viz::makeCameraPose(cam_pos, cam_pos, cam_y_dir);
    cv::Affine3f cam_pose(cv::Mat::eye(3, 3, CV_32FC1), cv::Vec3f(0, 0, 1));
    // カメラパラメータ
    cv::Matx33d K(intrinsic_matrix);

    /**
     * @brief 初期化関係
     * 
     */
    bool is_initialized = false;
    double match_rate_threshold = 0.3;
    dense_feature::feature_in_frame initialized_frame;
    cv::Mat rvec(3, 1, CV_64FC1), tvec(3, 1, CV_64FC1);

#define SHOW_RESULTS
#ifdef SHOW_RESULTS
    /**
     * @brief 3D描画の準備
     * 
     */
    cv::viz::Viz3d myWindow("Coordinate Frame");
    myWindow.showWidget("Coordinate Widget", cv::viz::WCoordinateSystem());
    cv::viz::WCameraPosition cpw(0.5); // Coordinate axes
    cv::viz::WCameraPosition wcamera(K, 1.0, cv::viz::Color::blue());
    cv::viz::WCameraPosition wcamera2(K, 1.0, cv::viz::Color::blue());
    cv::viz::WCameraPosition wcamera_cand1(K, 1.0, cv::viz::Color::yellow());
    cv::viz::WCameraPosition wcamera_cand2(K, 1.0, cv::viz::Color::yellow());
    cv::viz::WCameraPosition wcamera_cand3(K, 1.0, cv::viz::Color::yellow());
    cv::viz::WCameraPosition wcamera_cand4(K, 1.0, cv::viz::Color::yellow());
#endif

#ifdef REC
    cv::Mat tmp;
    double tst;
    lp_mav.get_frame_by_index(tmp, tst, 0);
    cv::VideoWriter wrt("test.mp4", cv::VideoWriter::fourcc('M', 'P', '4', 'V'), 30, tmp.size());
    for (size_t i = 0; i < 1000; i++)
#else
    for (size_t i = 0; i < lp_mav.get_frame_size(); i++)
#endif
    {
        /**
         * @brief 画像のキャプチャと補正
         * 
         */
        cv::Mat img, img_undistort, img_color;
        double tstamp;

        lp_mav.get_frame_by_index(img, tstamp, i);
        /**
         * @brief カメラ画像のレンズ歪みを補正する
         * @details
         * ここでの補正はレンズの円周方向、その接線方向のレンズ歪みになる。そのため、カメラのピンホールモデルの内部パラメータ（ここでのK）はしない。
         * 内部パラメータの画像中心なども補正されているわけではないので注意する。
         */
        cv::undistort(img, img_undistort, intrinsic_matrix, distortion_coeffs);
        /**
         * @brief 特徴点の検出と追跡を行う
         */
        dfe.detect_and_track(img_undistort, false);

        /**
         * @brief 描画用にモノクロ画像をRGB表現に変換する
         */
        cv::cvtColor(img_undistort * 0.5, img_color, CV_GRAY2BGR);

        /**
         * @brief 特徴点位置の初期化を行う
         * 
         */
        if (!is_initialized)
        {
            if (dfe.features.size() > 2)
            {
                dense_feature::feature_in_frame ref_frame, current_frame;
                // ref_frame = dfe.features[dfe.features.size() - ref_size - 1];
                ref_frame = dfe.features[1];
                current_frame = dfe.features[dfe.features.size() - 1];
                ref_frame.imageSizeWH = Eigen::Vector2i(img_color.size().width, img_color.size().height);
                ref_frame.intrinsic = intrinsic_eigen;
                current_frame.imageSizeWH = Eigen::Vector2i(img_color.size().width, img_color.size().height);
                current_frame.intrinsic = intrinsic_eigen;

                double match_rate = initializer::utils::initialize_feature_points(ref_frame, current_frame, initialized_frame);
                std::cout << "Match rate: " << match_rate << std::endl;

                if (match_rate > match_rate_threshold)
                {
                    is_initialized = true;
                    std::cout << "Initialized. Match rate : " << match_rate << std::endl;

                    // 特徴点位置を描画
                    std::vector<cv::Point3d> pointCloud;
                    for (size_t idx = 0; idx < initialized_frame.featuresIn3d.size(); idx++)
                    {
                        if (initialized_frame.featureMasks[idx])
                        {
                            pointCloud.emplace_back(
                                cv::Point3d(initialized_frame.featuresIn3d[idx][0],
                                            initialized_frame.featuresIn3d[idx][1],
                                            initialized_frame.featuresIn3d[idx][2]));
                        }
                    }
                    // 点群の描画
                    cv::viz::WCloud cloud(pointCloud);
                    myWindow.showWidget("CLOUD", cloud);

                    // カメラ移動量の描画
                    cv::Affine3d initial_cam_pose(cv::Mat::eye(3, 3, CV_64FC1), cv::Vec3f(0, 0, 0));
                    myWindow.showWidget("1", wcamera, initial_cam_pose);
                }
            }
        }
        else
        {
            auto &current_frame = dfe.features[dfe.features.size() - 1];

            // 3Dポイントと画像中の画素の対応点を作る
            std::vector<cv::Point3d> points_in_3d(0);
            std::vector<cv::Point2d> points_in_2d(0);
            std::set<uint64_t> points_in_3d_indices(initialized_frame.featureIDs.begin(), initialized_frame.featureIDs.end());
            std::set<uint64_t> points_in_2d_indices(current_frame.featureIDs.begin(), current_frame.featureIDs.end());
            std::set<uint64_t> indices_intersection;
            std::set_intersection(points_in_3d_indices.begin(), points_in_3d_indices.end(),
                                  points_in_2d_indices.begin(), points_in_2d_indices.end(),
                                  std::inserter(indices_intersection, indices_intersection.end()));
            for (size_t idx = 0; idx < initialized_frame.featuresIn3d.size(); idx++)
            {
                uint64_t feature_index = initialized_frame.featureIDs[idx];
                if (initialized_frame.featureMasks[idx])
                {
                    if (indices_intersection.count(feature_index) == 1)
                    {
                        points_in_3d.emplace_back(
                            cv::Point3d(initialized_frame.featuresIn3d[idx][0],
                                        initialized_frame.featuresIn3d[idx][1],
                                        initialized_frame.featuresIn3d[idx][2]));
                    }
                }
                else
                {
                    indices_intersection.erase(feature_index);
                }
            }
            for (size_t idx = 0; idx < current_frame.features.size(); idx++)
            {
                uint64_t feature_index = current_frame.featureIDs[idx];
                if (indices_intersection.count(feature_index) == 1)
                {
                    points_in_2d.emplace_back(
                        cv::Point2i(current_frame.features[idx][0],
                                    current_frame.features[idx][1]));
                }
            }

            // PNPを解く
            // cv::Mat rvec, tvec;
            cv::Mat out_points;
            cv::solvePnPRansac(points_in_3d, points_in_2d, K, {}, rvec, tvec, true, 1000, 16.0F, 0.9999, out_points, cv::SOLVEPNP_ITERATIVE);

            cv::Mat rotation_mat;
            cv::Rodrigues(rvec, rotation_mat); // 回転行列として復元

            // カメラ移動量の描画
            cv::Mat rot_draw, t_draw;
            rot_draw = rotation_mat.t();
            t_draw = tvec * -1.0;
            cv::Affine3d cam_pose(rot_draw, t_draw);
            myWindow.showWidget("2", wcamera2, cam_pose);

            /**
             * @brief 現在のカメラ位置に点群位置を再投影する
             * @details
             * - 入力データ
             * カメラ姿勢: rotation_mat, tvec
             * カメラの内部パラメータ: K
             */
            Eigen::Matrix3d cam_rotaion_mat;
            Eigen::Vector3d cam_translation_mat;
            cv::cv2eigen(rotation_mat.t(), cam_rotaion_mat);
            cv::cv2eigen(-tvec, cam_translation_mat);
            std::vector<cv::Point2d> reporojected_points;
            // 現在のカメラ位置から見た視点で特徴点位置を再投影する
            for (const auto &point_word_frame : points_in_3d)
            {
                Eigen::Vector2d point_device_frame = reproject_point(cam_rotaion_mat, cam_translation_mat,
                                                                     intrinsic_eigen,
                                                                     Eigen::Vector3d(point_word_frame.x, point_word_frame.y, point_word_frame.z));
                reporojected_points.emplace_back(cv::Point2d(point_device_frame[0], point_device_frame[1]));
            }
            for (size_t idx = 0; idx < reporojected_points.size(); idx++)
            {
                auto img_rect = cv::Rect(cv::Point(0, 0), img_color.size());
                if (img_rect.contains(reporojected_points[idx]))
                {
                    cv::line(img_color, points_in_2d[idx], reporojected_points[idx], cv::Scalar(255, 255, 255), 1);
                    cv::circle(img_color, reporojected_points[idx], 3, cv::Scalar(0, 255, 0), 1);
                }

                cv::circle(img_color, points_in_2d[idx], 3, cv::Scalar(255, 0, 0), 1);
            }
        }

#ifdef SHOW_RESULTS
        cv::imshow("feature", img_color);
        cv::waitKey(1);

        myWindow.spinOnce(1);
#endif

#ifdef REC
        wrt << img;
#endif
    }
}