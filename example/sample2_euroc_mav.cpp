#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/viz.hpp>
// #include "dense_feature_extructor_proto.hpp"
#include "dense_feature_extructor.hpp"

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

    // LogPlayer_extended lpe("/home/ery/Devel/tmp/assets/20191219_2/20191219_24", 0);

    // dense_feature_extructor dfe;
    dense_feature::dense_feature_extructor dfe(0.1, 0.1);

    // dfe.run_extruction("/home/ery/assets/20191219_2/20191219_31");
    // dfe.run_extruction("/home/ery/assets/20191219_1/20191219_2");
    // dfe.run_extruction("/home/ery/assets/20191115/20191115_40_2m_track");

    // dfe.run_extruction("/home/ery/Devel/tmp/assets/20191219_2/20191219_31");
    // dfe.run_extruction_cam("/dev/video0", 1.0);
    // dfe.run_extruction_cam("/home/ery/Devel/tmp/assets/IMG_5144.MOV", 1.0 / 2.0);

    // std::string path_to_log_dir = "/home/ery/assets/20191115/20191115_40_2m_track";
    // std::string path_to_log_dir = "/home/ery/Devel/tmp/assets/20191219_1/20191219_3";

    // /e/subspace/tmp/tmp/V1_01_easy/mav0/cam0
    // LogPlayer_euroc_mav lp_mav("/home/ery/Downloads/V1_01_easy/mav0/cam0", 0.001);
    // LogPlayer_euroc_mav lp_mav("/home/ery/Downloads/V2_01_easy/mav0/cam0", 0.001);
    LogPlayer_euroc_mav lp_mav("/e/subspace/tmp/tmp/V1_01_easy/mav0/cam0", 0.001);
    // LogPlayer_euroc_mav lp_mav("/e/subspace/tmp/tmp/MH_01_easy/mav0/cam0", 0.001);

    int64_t ref_size = 30;

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
    cv::Mat distortion_coeffs(5, 1, CV_32FC1);
    distortion_coeffs = (cv::Mat_<float>(5, 1) << -0.28340811, 0.07395907, 0.00019359, 1.76187114e-05);

    /**
     * @brief 3D描画の準備
     * 
     */
    cv::viz::Viz3d myWindow("Coordinate Frame");
    myWindow.showWidget("Coordinate Widget", cv::viz::WCoordinateSystem());
    cv::viz::WCameraPosition cpw(0.5); // Coordinate axes
    cv::Vec3f cam_pos(3.0f, 3.0f, 3.0f), cam_focal_point(3.0f, 3.0f, 2.0f), cam_y_dir(-1.0f, 0.0f, 0.0f);

    // cv::Affine3f cam_pose = cv::viz::makeCameraPose(cam_pos, cam_pos, cam_y_dir);
    cv::Affine3f cam_pose(cv::Mat::eye(3, 3, CV_32FC1), cv::Vec3f(0, 0, 1));
    // カメラパラメータ
    cv::Matx33d K(intrinsic_matrix);
    cv::Matx33d Kinv = K.inv();
    cv::viz::WCameraPosition wcamera(K, 1.0, cv::viz::Color::blue());
    cv::viz::WCameraPosition wcamera2(K, 1.0, cv::viz::Color::blue());
    cv::viz::WCameraPosition wcamera_cand1(K, 1.0, cv::viz::Color::yellow());
    cv::viz::WCameraPosition wcamera_cand2(K, 1.0, cv::viz::Color::yellow());
    cv::viz::WCameraPosition wcamera_cand3(K, 1.0, cv::viz::Color::yellow());
    cv::viz::WCameraPosition wcamera_cand4(K, 1.0, cv::viz::Color::yellow());

    cv::Mat current_position, current_attitude;
    current_position = (cv::Mat_<double>(3, 1) << 0, 0, 0);
    current_attitude = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);

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
        cv::Mat img, img_undistort, img_color;
        double tstamp;

        lp_mav.get_frame_by_index(img, tstamp, i);
        cv::undistort(img, img_undistort, intrinsic_matrix, distortion_coeffs);
        cv::cvtColor(img_undistort * 0.5, img_color, CV_GRAY2BGR);
        dfe.detect_and_track(img_undistort, false);

        /**
         * @brief 特徴点のトラッキング結果とIDの対応Mapを生成する
         * 
         */
        std::map<uint64_t, std::vector<cv::Point2i>> feature_lists;
        for (size_t i = 0; i < (dfe.features[dfe.features.size() - 1]).features.size(); i++)
        {
            auto &f = dfe.features[dfe.features.size() - 1];
            feature_lists[f.featureIDs[i]] = std::vector<cv::Point2i>({cv::Point2i(f.features[i][0], f.features[i][1])});
        }
        for (size_t fnum = dfe.features.size() - 1; fnum > std::max(static_cast<int64_t>(dfe.features.size()) - ref_size, 0l); fnum--)
        {
            for (size_t i = 0; i < dfe.features[fnum].features.size(); i++)
            {
                auto &f = dfe.features[fnum];
                if (feature_lists.count(f.featureIDs[i]) != 0)
                {
                    feature_lists[f.featureIDs[i]].emplace_back(cv::Point2i(f.features[i][0], f.features[i][1]));
                }
            }
        }

        /**
         * @brief 初期化処理をとりあえず実装してみる
         *
         */
        // 焦点距離とレンズ主点
        double fovx, fovy, focal, pasp;
        cv::Point2d pp;
        cv::calibrationMatrixValues(intrinsic_matrix_d, cv::Size(img_undistort.cols, img_undistort.rows), 0.0, 0.0, fovx, fovy, focal, pp, pasp);

        cv::Mat E, R, t, mask;
        cv::Mat R1, R2, trans;
        std::vector<cv::Point2f> prev_points(0), current_points(0), current_points_device(0), prev_points_normalized(0), current_points_normalized(0);
        if (dfe.features.size() >= 2)
        {
            // 前フレームと今のフレームの特徴点ペアを生成する
            int32_t num_ref_frames = 30;
            for (const auto &[id, p] : feature_lists)
            {
                if (p.size() >= num_ref_frames)
                {
                    cv::Mat p_tmp = cv::Mat_<double>(3, 1, CV_64FC1);
                    p_tmp = Kinv * (cv::Mat_<double>(3, 1, CV_64FC1) << p[0].x, p[0].y, 1.0);
                    current_points_normalized.emplace_back(cv::Point2f(p_tmp.at<double>(0), p_tmp.at<double>(1)));
                    current_points.emplace_back(p[0]);
                    current_points_device.emplace_back(p[0]);

                    p_tmp = Kinv * (cv::Mat_<double>(3, 1, CV_64FC1) << p[num_ref_frames - 1].x, p[num_ref_frames - 1].y, 1.0);
                    prev_points_normalized.emplace_back(cv::Point2f(p_tmp.at<double>(0), p_tmp.at<double>(1)));
                    prev_points.emplace_back(cv::Point2f(p[num_ref_frames - 1].x, p[num_ref_frames - 1].y));
                }
            }

            if (current_points.size() >= num_ref_frames)
            {
                E = cv::findEssentialMat(prev_points, current_points, focal, pp, cv::RANSAC, 0.999, 0.99, mask);
                // E = cv::findEssentialMat(prev_points, current_points, focal, pp, cv::LMEDS, 0.999, 0.99, mask);
                // cv::decomposeEssentialMat(E, R1, R2, trans);

                cv::recoverPose(E, prev_points, current_points, R, t, focal, pp, mask);
                std::cout << R << std::endl;
                std::cout << t << std::endl;

                current_attitude = R;
                current_position = t;

                // R = R.t();
                // t = -1.0 * t;
                // std::cout << mask << std::endl;

                double matching_rate = static_cast<double>(cv::countNonZero(mask)) / prev_points.size();
                std::cout << "Match rate: " << matching_rate << std::endl;
                if (matching_rate > 0.1)
                {
                    //正規化座標系で計算しているのでProjection matrix=Extrinsic camera parameter matrix
                    cv::Mat prjMat1, prjMat2;
                    prjMat1 = cv::Mat::eye(3, 4, CV_64FC1); //片方は回転、並進ともに0
                    prjMat2 = cv::Mat(3, 4, CV_64FC1);
                    for (int ii = 0; ii < 3; ++ii)
                    {
                        for (int j = 0; j < 3; ++j)
                        {
                            prjMat2.at<double>(ii, j) = current_attitude.at<double>(ii, j);
                        }
                    }
                    prjMat1 = intrinsic_matrix_d * prjMat1;

                    prjMat2.at<double>(0, 3) = current_position.at<double>(0);
                    prjMat2.at<double>(1, 3) = current_position.at<double>(1);
                    prjMat2.at<double>(2, 3) = current_position.at<double>(2);
                    prjMat2 = intrinsic_matrix_d * prjMat2;

                    std::cout << "Projection Matrix 1:\n"
                              << prjMat1 << std::endl;
                    std::cout << "Projection Matrix 2:\n"
                              << prjMat2 << std::endl;

                    // 三角測量による三次元位置の推定
                    std::vector<cv::Point2d> cam0pnts;
                    std::vector<cv::Point2d> cam1pnts;
                    std::vector<int32_t> point_indices;
                    for (size_t idx = 0; idx < prev_points.size(); idx++)
                    {
                        if (mask.at<unsigned char>(idx) > 0)
                        {
                            cam0pnts.emplace_back(prev_points[idx].x, prev_points[idx].y);
                            cam1pnts.emplace_back(current_points[idx].x, current_points[idx].y);
                            point_indices.emplace_back(idx);
                        }
                    }
                    cv::Mat pnts3D(4, cam0pnts.size(), CV_64F);
                    cv::triangulatePoints(prjMat1, prjMat2, cam0pnts, cam1pnts, pnts3D);
                    std::vector<cv::Point3d> pointCloud;

                    for (int ii = 0; ii < pnts3D.cols; ++ii)
                    {
                        pointCloud.emplace_back(cv::Point3d(pnts3D.at<double>(0, ii),
                                                            pnts3D.at<double>(1, ii),
                                                            pnts3D.at<double>(2, ii)) /
                                                pnts3D.at<double>(3, ii));
                        double pix_color = img.at<uint8_t>(current_points_device[point_indices[ii]].y, current_points_device[point_indices[ii]].x);
                    }

                    // 点群の描画
                    cv::viz::WCloud cloud(pointCloud);
                    myWindow.showWidget("CLOUD", cloud);

                    // カメラ移動量の描画
                    cv::Affine3d initial_cam_pose(cv::Mat::eye(3, 3, CV_64FC1), cv::Vec3f(0, 0, 0));
                    cv::Affine3d current_cam_pose(current_attitude.t(), cv::Vec3f(current_position) * -1.0);
                    myWindow.showWidget("1", wcamera2, initial_cam_pose);
                    myWindow.showWidget("2", wcamera, current_cam_pose);
                }
            }

            // 対応した特徴点の描画
            for (size_t j = 0; j < current_points_device.size(); j++)
            {
                if (mask.at<int32_t>(j, 0))
                {
                    cv::circle(img_color, current_points_device[j], 2, cv::Scalar(255, 0, 255));
                }
                else
                {
                    cv::circle(img_color, prev_points[j], 2, cv::Scalar(0, 0, 255));
                }
            }
        }

        // /**
        //  * @brief トラッキング長を計算する
        //  *
        //  */
        // double maxlen = 0;
        // std::vector<double> lens(0);
        // for (const auto &[id, p] : feature_lists)
        // {
        //     cv::Point2i d = p[0] - p[p.size() - 1];
        //     double dist = cv::norm(d) / p.size();

        //     lens.emplace_back(cv::norm(dist));
        // }
        // double len_sum = std::accumulate(std::begin(lens), std::end(lens), 0.0);
        // double len_ave = len_sum / lens.size();
        // double len_var = std::inner_product(std::begin(lens), std::end(lens), std::begin(lens), 0.0) / lens.size() - len_ave * len_ave;
        // for (const auto l : lens)
        // {
        //     if ((maxlen < l) && (l < (1.0 * std::sqrt(len_var)) + len_ave))
        //     {
        //         maxlen = l;
        //     }
        // }

        // for (const auto &[id, p] : feature_lists)
        // {
        //     cv::Point2i d = p[0] - p[p.size() - 1];
        //     double angle = std::atan2(d.y, d.x) * 180.0 / M_PI;
        //     double len = cv::norm(d) / p.size();
        //     angle += 180;

        //     // cv::Scalar dcolor = HSVtoRGB(angle, 1, 1);
        //     cv::Scalar dcolor = HSVtoRGB(len / maxlen * 360.0, 1, 1);
        //     // cv::Scalar dcolor = cv::Scalar(255, 255, 255);
        //     // cv::Scalar dcolor = colors[id % num_colors];

        //     if (len < (2.0 * std::sqrt(len_var) + len_ave))
        //     {
        //         // cv::polylines(img_color, p, false, dcolor, 1);
        //         // cv::polylines(img_color, p, false, cv::Scalar(255, 255, 255), 1);
        //         // cv::circle(img_color, p[0], 2, dcolor, 1);
        //     }
        // }

        cv::imshow("feature", img_color);
        cv::waitKey(1);

        myWindow.spinOnce(1);

#ifdef REC
        wrt << img;
#endif
    }
}