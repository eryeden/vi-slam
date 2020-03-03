#include <iostream>
#include <algorithm>

#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/viz.hpp>

#include <Eigen/Core>
#include <opencv2/core/eigen.hpp>

#include "frame.hpp"
#include "landmark.hpp"
#include "camera.hpp"

#include "ba_pre.hpp"

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



/**
 * @brief BA処理の前段として、各フレームのPose、特徴点位置の初期化、Jacobian、Hessianの計算までを実装する
 * @details
 * ## 実装内容
 * - 全フレーム対象のFrame Poseと特徴点位置の初期化
 * - Jacobian, Hessianの生成と、非Zero要素を画像として表示する
 */
int main()
{

    uint32_t num_colors = 1000;
    std::vector<cv::Scalar> colors(num_colors);
    for (size_t i = 0; i < colors.size(); i++)
    {
        double h, s, v;
        h = static_cast<double>(i) / static_cast<double>(num_colors) * 360.0;
        colors[i] = HSVtoRGB(h, 1, 1);
    }

    dense_feature::dense_feature_extructor dfe(0.1, 0.1);

    LogPlayer_euroc_mav lp_mav("/home/ery/Downloads/V1_01_easy/mav0/cam0", 0.001);
    // LogPlayer_euroc_mav lp_mav("/home/ery/Downloads/V2_01_easy/mav0/cam0", 0.001);
//     LogPlayer_euroc_mav lp_mav("/e/subspace/tmp/tmp/V1_01_easy/mav0/cam0", 0.001);
    // LogPlayer_euroc_mav lp_mav("/e/subspace/tmp/tmp/MH_01_easy/mav0/cam0", 0.001);
//    LogPlayer_euroc_mav lp_mav("/home/ery/assets/V1_01_easy/mav0/cam0", 0.001);

    // // カメラ画像を補正するようにする
    // // カメラの歪み補正 パラメータ FIXME 外用Econカメラの4:3画像サイズの補正用パラメータなので、カメラでパラメータを変更できるようにしなければならない
    cv::Mat intrinsic_matrix(3, 3, CV_32FC1);
    intrinsic_matrix = (cv::Mat_<float>(3, 3) << 458.654, 0.0000000000000000e+00, 367.215,
                        0.0000000000000000e+00, 457.296, 248.375,
                        0.0000000000000000e+00, 0.0000000000000000e+00, 1.0000000000000000e+00);
    Eigen::Matrix3d intrinsic_eigen;
    cv::cv2eigen(intrinsic_matrix, intrinsic_eigen);

    cv::Mat distortion_coeffs(5, 1, CV_32FC1);
    distortion_coeffs = (cv::Mat_<float>(5, 1) << -0.28340811, 0.07395907, 0.00019359, 1.76187114e-05);
    std::vector<double> distortion_coeffs_array = { -0.28340811, 0.07395907, 0.00019359, 1.76187114e-05};

    cv::Vec3f cam_pos(3.0f, 3.0f, 3.0f), cam_focal_point(3.0f, 3.0f, 2.0f), cam_y_dir(-1.0f, 0.0f, 0.0f);

    // cv::Affine3f cam_pose = cv::viz::makeCameraPose(cam_pos, cam_pos, cam_y_dir);
    cv::Affine3f cam_pose(cv::Mat::eye(3, 3, CV_32FC1), cv::Vec3f(0, 0, 1));
    // カメラパラメータ
    cv::Matx33d K(intrinsic_matrix);

    //! カメラパラメータをまとめて初期化する
    cv::Mat dummy_image;
    double dummy_timestamp;
    lp_mav.get_frame_by_index(dummy_image, dummy_timestamp, 0);
//    vislam::data::camera camera_pram(0,
//                                     dummy_image.size().width, dummy_image.size().height,
//                                     30,
//                                     intrinsic_eigen(0,0), intrinsic_eigen(1,1), intrinsic_eigen(2,0), intrinsic_eigen(2,1),
//                                     distortion_coeffs_array[0], distortion_coeffs_array[1], distortion_coeffs_array[2], distortion_coeffs_array[3], distortion_coeffs_array[4]);

    vislam::data::camera camera_pram(0,
                                     dummy_image.size().width, dummy_image.size().height,
                                     30,
                                     458.654, 457.296, 367.215, 248.375, //! eigenのmatからだとうまいこといかなかった
                                     distortion_coeffs_array[0], distortion_coeffs_array[1], distortion_coeffs_array[2], distortion_coeffs_array[3], distortion_coeffs_array[4]);

    /**
     * @brief 初期化関係
     * 
     */
    bool is_initialized = false;
    double match_rate_threshold = 0.5;
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

    cv::viz::WCameraPosition wcamera_current_pnp(K, 1.0, cv::viz::Color::green());
#endif


    /**
     * @brief 観測したフレームの保存場所
     */
    std::unordered_map<uint64_t , vislam::data::frame> database_frame;

    /**
     * @brief 観測した特徴点（Feature point, landmark）の保存場所
     */
    std::unordered_map<uint64_t, vislam::data::landmark> database_landmark;


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
         * @brief 抽出した特徴点からdata::frameを生成する
         */
        vislam::data::frame frame_current;
        /**
         * @brief 今回のフレームで抽出・トラックできた特徴点を挿入する。
         */
        auto feature_points_current = dfe.features[dfe.features.size()-1];
        for(size_t index_current_feature = 0; index_current_feature < feature_points_current.featureIDs.size(); index_current_feature++){

            /**
             * @brief Frame中に検出した特徴点を記録する
             */
            uint64_t current_feature_id = feature_points_current.featureIDs[index_current_feature];
            Eigen::Vector2i  current_feature_position_in_device = feature_points_current.features[index_current_feature];
            frame_current.observingFeaturePointInDevice[current_feature_id] ={current_feature_position_in_device[0], current_feature_position_in_device[1]};
            frame_current.observingFeatureId.emplace(current_feature_id);

            /**
             * @brief 特徴点データベースに登録する
             */
            if(database_landmark.count(current_feature_id) !=0){ // databaseに登録済みの場合
                database_landmark[current_feature_id].isTracking = true;
                /**
                 * @brief std::setへの要素追加が発生している。ここでの処理コストが大きい場合はstd::vectorへの置き換えが必要かもしれない。
                 * 結局、カメラIDは増加しかしないため、std::vectorで追加しつつ、std::set_intersectionを行っても同じなのかもしれない。
                 */
                database_landmark[current_feature_id].observedFrameId.emplace(i);
            }else{ // databaseに登録なしの場合
                database_landmark[current_feature_id] = vislam::data::landmark(current_feature_id,
                        {i},
                        {0,0,0},
                        false,
                        true,
                        false
                        );
            }
        }

        /**
         * @brief カメラパラメータ関係の初期化を行う
         */
        frame_current.cameraParameter = camera_pram;

        /**
         * @brief Frameのデータベースに登録する
         */
        database_frame[i] = frame_current;
//        std::cout << "Num frames: " << database_frame.size() << std::endl; // フレーム数はunordered_mapのsizeで得られるぽい

        /**
         * @brief 特徴点位置の初期化を行う
         * 
         */
        if(!is_initialized){
            if(database_frame.size()> 2){

                /**
                 * @brief 特徴点位置の初期化を試みる
                 */
                const auto & reference_frame = database_frame[1]; // index:0のframeは何も入っていなかったの注意
                const auto & current_frame = database_frame[i];
                std::vector<vislam::data::landmark> localized_landmarks;

                vislam::Vec3_t initialized_position_current_frame;
                vislam::Quat_t initialized_attitude_current_frame;

//                double match_rate = initializer::utils::initialize_feature_points(reference_frame, current_frame, localized_landmarks);
                double match_rate = initializer::utils::initialize_feature_points(reference_frame, current_frame, localized_landmarks,
                        initialized_position_current_frame,
                        initialized_attitude_current_frame);

                std::cout << "Match rate: " << match_rate << std::endl;

                if(match_rate > match_rate_threshold){

                    /**
                     * @brief 特徴点位置の初期化を完了して特徴点データベースの更新を行う
                     */
                    is_initialized = true;
                    std::cout << "Initialized. Match rate : " << match_rate << std::endl;

                    /**
                     * @brief 特徴点の観測情報を更新する
                     * @details
                     * ここではLandmark databaseの位置情報、初期化情報などを更新している。
                     * なので、次からはLandmarkDatabaseを参照することで初期化結果を利用できる
                     */
                     for(const auto & lm :  localized_landmarks){
                         auto & related_landmark = database_landmark[lm.id];
                         related_landmark.isTracking = lm.isTracking;
                         related_landmark.isOutlier = lm.isOutlier;
                         related_landmark.isInitialized = lm.isInitialized;
                         related_landmark.positionInWorld = lm.positionInWorld;
                         related_landmark.id = lm.id;
                     }

                     /**
                      * @brief 今回の初期化フレームの位置を設定する
                      */
                    auto & ref_current_frame = database_frame[i];
                    ref_current_frame.cameraPosition = initialized_position_current_frame;
                    ref_current_frame.cameraAttitude = initialized_attitude_current_frame;

                    // 特徴点位置を描画
                    std::vector<cv::Point3d> pointCloud;
                    for (auto & localized_landmark : localized_landmarks)
                    {
                        if (!localized_landmark.isOutlier)
                        {
                            pointCloud.emplace_back(
                                    cv::Point3d(localized_landmark.positionInWorld[0],
                                                localized_landmark.positionInWorld[1],
                                                localized_landmark.positionInWorld[2]));
                        }
                    }
                    // 点群の描画
                    cv::viz::WCloud cloud(pointCloud);
                    myWindow.showWidget("CLOUD", cloud);

                    // 基準カメラ位置の描画
                    cv::Affine3d initial_cam_pose(cv::Mat::eye(3, 3, CV_64FC1), cv::Vec3f(0, 0, 0));
                    myWindow.showWidget("1", wcamera, initial_cam_pose);
                    // 初期化カメラ位置の描画
                    cv::Mat current_camera_attitude;
                    cv::eigen2cv(ref_current_frame.cameraAttitude.toRotationMatrix(), current_camera_attitude);
                    cv::Affine3d current_cam_pose(current_camera_attitude, cv::Vec3f(ref_current_frame.cameraPosition[0], ref_current_frame.cameraPosition[1], ref_current_frame.cameraPosition[2]));
                    myWindow.showWidget("2", wcamera_cand1, current_cam_pose);
                }
            }
        }

        /**
         * @brief 初期化後の処理
         * @details
         * 処理の内容
         * 1. 新規フレーム位置の初期化
         * 2. 初期化対象のLandmarkを選択、Landmark位置の初期化を実施
         * 3. BA処理の前段階として、Jacobian,Hessianを生成
         * 4. BA実施
         * 5. アウトライアの除去を実施
         */
        if(is_initialized){


            /**
             * @brief 1. current frameの位置、姿勢を初期化
             */
            vislam::Vec3_t  position_current_frame;
            vislam::Quat_t attitude_current_frame;
            initializer::utils::estimate_frame_pose_pnp(frame_current, database_landmark, position_current_frame, attitude_current_frame);
            //! p3pで計算した現在フレームの位置をDataBaseに登録する
            database_frame[i].cameraPosition = position_current_frame;
            database_frame[i].cameraAttitude = attitude_current_frame;

            //! 推定カメラ位置の描画
            cv::Mat current_camera_attitude;
            cv::eigen2cv(database_frame[i].cameraAttitude.toRotationMatrix(), current_camera_attitude);
            cv::Affine3d current_cam_pose(current_camera_attitude, cv::Vec3f(database_frame[i].cameraPosition[0], database_frame[i].cameraPosition[1], database_frame[i].cameraPosition[2]));
            myWindow.showWidget("estacam", wcamera_current_pnp, current_cam_pose);

            /**
             * @brief 2. Landmark位置の初期化を実施する
             * @details
             * 基本、初期化（２視点から観測されている特徴点位置を三角測量で求める）は次の条件の特徴点に対して実施する。
             * - 初期化されていない
             * - 2 Frame以上観測されている
             * - 観測されているフレームの最大視差が、ある値以上になっている
             * - 今回のフレームで観測されている
             */




        }



#ifdef SHOW_RESULTS
        cv::imshow("feature", img_color);
        cv::waitKey(1);
//
        myWindow.spinOnce(1);
#endif

#ifdef REC
        wrt << img;
#endif
    }
}