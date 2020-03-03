#include "initializer.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <opencv2/core/eigen.hpp>

//#include <chrono>

using namespace initializer;

/**
 * @brief 特徴点位置の初期化を行う
 * 
 * @param features_reference 
 * @param features_current 
 * @return dense_feature::feature_in_frame 
 */
double utils::initialize_feature_points(const dense_feature::feature_in_frame &features_reference,
                                        const dense_feature::feature_in_frame &features_current,
                                        dense_feature::feature_in_frame &features_output)

{
    // 出力データ
    dense_feature::feature_in_frame out_frame = features_current;
    std::vector<Eigen::Vector3d> feature_points_in_3d_current_frame(features_current.features.size(), Eigen::Vector3d::Zero());
    std::vector<int8_t> feature_mask(features_current.features.size(), 0);

    // カメラ関係パラメータ初期化
    cv::Mat intrinsic(3, 3, CV_64FC1);
    cv::eigen2cv(features_current.intrinsic, intrinsic);

    double fovx,
        fovy, focal, pasp;
    cv::Point2d pp;
    cv::calibrationMatrixValues(intrinsic, cv::Size(features_current.imageSizeWH[0], features_current.imageSizeWH[1]), 0.0, 0.0, fovx, fovy, focal, pp, pasp);

    /**
     * @brief 特徴点のトラッキング結果とIDの対応Mapを生成する
     * 
     */
    std::map<uint64_t, std::vector<cv::Point2i>> feature_lists;
    std::map<uint64_t, uint64_t> feature_index_map;
    for (size_t i = 0; i < features_current.features.size(); i++)
    {
        auto &f = features_current;
        feature_lists[f.featureIDs[i]] = std::vector<cv::Point2i>({cv::Point2i(f.features[i][0], f.features[i][1])});
        feature_index_map[f.featureIDs[i]] = i;
    }
    for (size_t i = 0; i < features_reference.features.size(); i++)
    {
        auto &f = features_reference;
        if (feature_lists.count(f.featureIDs[i]) != 0)
        {
            feature_lists[f.featureIDs[i]].emplace_back(cv::Point2i(f.features[i][0], f.features[i][1]));
        }
    }
    //特徴点ペアを設定する
    std::vector<cv::Point2d> feature_points_reference(0), feature_points_current(0);
    std::vector<uint64_t> matching_feature_ids(0);   // 現在フレームとリファレンスフレーム両方に存在する特徴点のペアのID
    std::vector<size_t> matching_feature_indices(0); // 現在フレームとリファレンスフレーム両方に存在する特徴点のペアのID
    for (const auto &[id, f] : feature_lists)
    {
        if (f.size() == 2)
        {
            matching_feature_ids.emplace_back(id);
            feature_points_current.emplace_back(f[0]);
            feature_points_reference.emplace_back(f[1]);
        }
    }

    /**
     * @brief 5点法でカメラ位置を推定、特徴点位置を計算する
     * 
     */
    cv::Mat E, R, t, mask;                // 推定したカメラの移動量がここに入る
    if (matching_feature_ids.size() >= 5) // マッチ対象の特徴点が５点以上の場合
    {
        // std::cout << "input features: " << features_current.features.size() << std::endl;

        // 5点法でEssentialMatrixを推定する
        E = cv::findEssentialMat(feature_points_reference, feature_points_current, focal, pp, cv::RANSAC, 0.999, 0.99, mask);
        // double matching_rate = static_cast<double>(cv::countNonZero(mask)) / feature_points_reference.size();
        cv::recoverPose(E, feature_points_reference, feature_points_current, R, t, focal, pp, mask);

        // //正規化座標系で計算しているのでProjection matrix=Extrinsic
        cv::Mat prjMat1(3, 4, CV_64FC1), prjMat2(3, 4, CV_64FC1);
        prjMat1 = cv::Mat::eye(3, 4, CV_64FC1); //片方は回転、並進ともに0
        for (int ii = 0; ii < 3; ++ii)
        {
            for (int j = 0; j < 3; ++j)
            {
                prjMat2.at<double>(ii, j) = R.at<double>(ii, j);
            }
        }
        prjMat1 = intrinsic * prjMat1;

        prjMat2.at<double>(0, 3) = t.at<double>(0);
        prjMat2.at<double>(1, 3) = t.at<double>(1);
        prjMat2.at<double>(2, 3) = t.at<double>(2);
        prjMat2 = intrinsic * prjMat2;

        // // 三角測量による三次元位置の推定
        std::vector<cv::Point2d> cam0pnts(feature_points_reference.size(), cv::Point2d(0, 0));
        std::vector<cv::Point2d> cam1pnts(feature_points_reference.size(), cv::Point2d(0, 0));
        for (size_t idx = 0; idx < feature_points_reference.size(); idx++)
        {
            cam0pnts[idx] = cv::Point2d(feature_points_reference[idx].x, feature_points_reference[idx].y);
            cam1pnts[idx] = cv::Point2d(feature_points_current[idx].x, feature_points_current[idx].y);

            feature_mask[feature_index_map[matching_feature_ids[idx]]] = mask.at<unsigned char>(idx);
        }
        cv::Mat pnts3D(4, cam0pnts.size(), CV_64F);
        /**
         * @brief 三角測量で特徴点位置を計算する
         * @details
         * このときの特徴点位置座標系はreferenceフレームのカメラ位置を原点/直交基底としている。
         * そのため、currentフレームのカメラ位置を原点とした座標系に変換して出力する必要がある。
         *
         */
        cv::triangulatePoints(prjMat1, prjMat2, cam0pnts, cam1pnts, pnts3D);

        // 特徴点位置の座標系をCurrentFrameのカメラ位置を原点/直交基底にしたものとして変換する
        for (int ii = 0; ii < pnts3D.cols; ++ii)
        {
            cv::Point3d pos = cv::Point3d(pnts3D.at<double>(0, ii),
                                          pnts3D.at<double>(1, ii),
                                          pnts3D.at<double>(2, ii)) /
                              pnts3D.at<double>(3, ii);
            cv::Mat pos_mod = R * (cv::Mat_<double>(3, 1) << pos.x, pos.y, pos.z) + t;
            feature_points_in_3d_current_frame[feature_index_map[matching_feature_ids[ii]]] =
                Eigen::Vector3d(pos_mod.at<double>(0), pos_mod.at<double>(1), pos_mod.at<double>(2));
        }

        // cv::cv2eigen(R, out_frame.rotation);
        // out_frame.translation << t.at<double>(0), t.at<double>(1), t.at<double>(2);
        out_frame.rotation << 1, 0, 0, 0, 1, 0, 0, 0, 1;
        out_frame.translation << 0, 0, 0;

        out_frame.featuresIn3d = feature_points_in_3d_current_frame;
        out_frame.featureMasks = feature_mask;
        features_output = out_frame;

        double matching_rate = static_cast<double>(cv::countNonZero(mask)) / feature_points_current.size();
        return matching_rate;
    }
    else // 5点以下の場合
    {
        return 0;
    }
}




double
utils::initialize_feature_points(const vislam::data::frame &frame_reference,
                                 const vislam::data::frame &frame_current,
                                 std::vector<vislam::data::landmark> &initialized_landmarks,
                                 vislam::Vec3_t & current_frame_position_in_world,
                                 vislam::Quat_t & current_frame_attitude_in_world)
{
    // カメラ関係パラメータ初期化
    const auto &camera_param = frame_current.cameraParameter;
    cv::Mat intrinsic(3, 3, CV_64FC1);
    cv::eigen2cv(camera_param.get_intrinsic_matrix(), intrinsic);
    double fovx, fovy, focal, pasp;
    cv::Point2d pp;
    cv::calibrationMatrixValues(intrinsic, cv::Size(camera_param.width, camera_param.height), 0.0, 0.0, fovx, fovy, focal, pp, pasp);

    /**
     * @brief ReferenceとCurrent同時に観測しているFeature IDを抽出する
     * @details
     * std::unordered_setを活用することで、高速な探索を実現する。
     */
    const auto & reference_feature_ids = frame_reference.observingFeatureId;
    std::vector<uint64_t> ref_feature_ids_vector(reference_feature_ids.begin(), reference_feature_ids.end());
    const auto & current_feature_ids = frame_current.observingFeatureId;
    std::vector<uint64_t >curt_feature_ids_vector(current_feature_ids.begin(), current_feature_ids.end());

    std::vector<uint64_t > intersection_feature_ids;
    std::set_intersection(ref_feature_ids_vector.begin(), ref_feature_ids_vector.end(),
                          curt_feature_ids_vector.begin(), curt_feature_ids_vector.end(),
                          std::inserter(intersection_feature_ids, intersection_feature_ids.end()));

    /**
     * @brief 積集合の要素を集める。
     */
    std::vector<cv::Point2d> feature_points_reference(0), feature_points_current(0);
    std::vector<uint64_t> feature_points_ids(0);
    feature_points_current.reserve(intersection_feature_ids.size());
    feature_points_reference.reserve(intersection_feature_ids.size());
    feature_points_ids.reserve(intersection_feature_ids.size());
    for(const auto idx : intersection_feature_ids){
        feature_points_current.emplace_back(
                cv::Point2d(frame_current.observingFeaturePointInDevice.at(idx)[0],
                            frame_current.observingFeaturePointInDevice.at(idx)[1]));
        feature_points_reference.emplace_back(
                cv::Point2d(frame_reference.observingFeaturePointInDevice.at(idx)[0],
                            frame_reference.observingFeaturePointInDevice.at(idx)[1]));
        feature_points_ids.emplace_back(idx);

//        std::cout << "Intersection: " << idx << std::endl;
    }




    /**
     * @brief 5点法でカメラ位置を推定、特徴点位置を計算する
     *
     */
    cv::Mat E, R, t, mask;                //! 推定したカメラの移動量がここに入る
    std::vector<vislam::data::landmark> output_landmarks; //! Currentカメラ座標系において位置推定された特徴点が入る
    if (intersection_feature_ids.size() >= 5) // マッチ対象の特徴点が５点以上の場合
    {
        // std::cout << "input features: " << features_current.features.size() << std::endl;

        // 5点法でEssentialMatrixを推定する
        E = cv::findEssentialMat(feature_points_reference, feature_points_current, focal, pp, cv::RANSAC, 0.999, 0.99,
                                 mask);
        // double matching_rate = static_cast<double>(cv::countNonZero(mask)) / feature_points_reference.size();
        cv::recoverPose(E, feature_points_reference, feature_points_current, R, t, focal, pp, mask);

        // //正規化座標系で計算しているのでProjection matrix=Extrinsic
        cv::Mat prjMat1(3, 4, CV_64FC1), prjMat2(3, 4, CV_64FC1);
        prjMat1 = cv::Mat::eye(3, 4, CV_64FC1); //片方は回転、並進ともに0
        for (int ii = 0; ii < 3; ++ii) {
            for (int j = 0; j < 3; ++j) {
                prjMat2.at<double>(ii, j) = R.at<double>(ii, j);
            }
        }
        prjMat1 = intrinsic * prjMat1;

        prjMat2.at<double>(0, 3) = t.at<double>(0);
        prjMat2.at<double>(1, 3) = t.at<double>(1);
        prjMat2.at<double>(2, 3) = t.at<double>(2);
        prjMat2 = intrinsic * prjMat2;

        // // 三角測量による三次元位置の推定
        std::vector<cv::Point2d> cam0pnts(feature_points_reference.size(), cv::Point2d(0, 0));
        std::vector<cv::Point2d> cam1pnts(feature_points_reference.size(), cv::Point2d(0, 0));
        std::vector<uint64_t> inlier_feature_ids(0); //! inlierの特徴点IDが入る
        for (size_t idx = 0; idx < feature_points_reference.size(); idx++) {
            cam0pnts[idx] = cv::Point2d(feature_points_reference[idx].x, feature_points_reference[idx].y);
            cam1pnts[idx] = cv::Point2d(feature_points_current[idx].x, feature_points_current[idx].y);
            if (mask.at<unsigned char>(idx)) { inlier_feature_ids.emplace_back(feature_points_ids[idx]); }

        }
        cv::Mat pnts3D(4, cam0pnts.size(), CV_64F);
        /**
         * @brief 三角測量で特徴点位置を計算する
         * @details
         * このときの特徴点位置座標系はreferenceフレームのカメラ位置を原点/直交基底としている。
         * そのため、currentフレームのカメラ位置を原点とした座標系に変換して出力する必要がある。
         *
         */
        cv::triangulatePoints(prjMat1, prjMat2, cam0pnts, cam1pnts, pnts3D);

        // 特徴点位置の座標系をCurrentFrameのカメラ位置を原点/直交基底にしたものとして変換する
        output_landmarks.reserve(pnts3D.cols);
        for (int ii = 0; ii < pnts3D.cols; ++ii) {
            cv::Point3d pos = cv::Point3d(pnts3D.at<double>(0, ii),
                                          pnts3D.at<double>(1, ii),
                                          pnts3D.at<double>(2, ii)) /
                              pnts3D.at<double>(3, ii);
            //! Currentカメラ座標系における特徴点位置を
            cv::Mat pos_mod = (cv::Mat_<double>(3, 1) << pos.x, pos.y, pos.z);

            bool is_outliear = mask.at<unsigned char>(ii) == 0;

            //! ここでは初期化済みの特徴点情報を生成しているのでis_initialized=trueになっている
            vislam::data::landmark current_landmark(feature_points_ids[ii],
                                                    {},
                                                    vislam::Vec3_t (pos_mod.at<double>(0), pos_mod.at<double>(1), pos_mod.at<double>(2)),
                                                    is_outliear,
                                                    true,
                                                    true);
            output_landmarks.emplace_back(current_landmark);
        }

        /**
         * @brief reference座標系での特徴点位置が出力される
         */
        initialized_landmarks = output_landmarks;

        /**
         * @brief Current frameの位置、姿勢を出力
         */
        vislam::Mat33_t current_attitude;
        vislam::Vec3_t current_position;
        cv::cv2eigen(R.t(), current_attitude); //! 転置が必要
        current_position << -t.at<double>(0), -t.at<double>(1), -t.at<double>(2); //! 逆なので、マイナスをつける
        current_frame_position_in_world = current_position;
        current_frame_attitude_in_world = vislam::Quat_t(current_attitude);


        double matching_rate = static_cast<double>(cv::countNonZero(mask)) / feature_points_current.size();
        return matching_rate;
    }else{
        return 0;
    }
}

double
utils::initialize_feature_points(const vislam::data::frame &frame_reference,
                                 const vislam::data::frame &frame_current,
                                 std::vector<vislam::data::landmark> &initialized_landmarks) {
    vislam::Vec3_t dummy_position;
    vislam::Quat_t dummy_attitude;
    initialize_feature_points(frame_reference, frame_current, initialized_landmarks, dummy_position, dummy_attitude);
}

double utils::estimate_frame_pose_pnp(const vislam::data::frame &frame_current,
                                      const std::unordered_map<uint64_t, vislam::data::landmark> &database_landmark,
                                      vislam::Vec3_t &current_frame_position_in_world,
                                      vislam::Quat_t &current_frame_attitude_in_world) {

    /**
     * @brief 実装方針
     * @details
     * 入力Frameで観測されたランドマークのうち、位置が求められていて（Initializeされている）、Outlierでないものとの対応をとる。
     */

    // 3Dポイントと画像中の画素の対応点を作る
    std::vector<cv::Point3d> points_in_3d(0);
    std::vector<cv::Point2d> points_in_2d(0);

    for(const auto id : frame_current.observingFeatureId){
        const auto & lm = database_landmark.at(id);
        if(lm.isInitialized && (!lm.isOutlier)){
            const auto feature_position_in_device = frame_current.observingFeaturePointInDevice.at(id);
            points_in_2d.emplace_back(cv::Point2d(feature_position_in_device[0], feature_position_in_device[1]));
            const auto feature_position_in_world = lm.positionInWorld;
            points_in_3d.emplace_back(cv::Point3d(feature_position_in_world[0], feature_position_in_world[1], feature_position_in_world[2]));
        }
    }

    // PNPを解く
    cv::Mat rvec, tvec;
    cv::Mat out_points;
    cv::Mat K;
    cv::eigen2cv(frame_current.cameraParameter.get_intrinsic_matrix(), K);
    cv::solvePnPRansac(points_in_3d, points_in_2d, K, {}, rvec, tvec, true, 1000, 8.0F, 0.9999, out_points, cv::SOLVEPNP_ITERATIVE);

    cv::Mat rotation_mat;
    cv::Rodrigues(rvec, rotation_mat); // 回転行列として復元

    cv::Mat rot_inv, t_inv;
    rot_inv = rotation_mat.t();
    t_inv = tvec * -1.0;

    /**
     * @brief 出力する
     */
    vislam::Mat33_t eigen_rotation_mat;
    cv::cv2eigen(rot_inv, eigen_rotation_mat);
    current_frame_attitude_in_world = vislam::Quat_t(eigen_rotation_mat);
    current_frame_position_in_world << t_inv.at<double>(0), t_inv.at<double>(1),t_inv.at<double>(2);

    return 0;
}

std::vector<uint64_t> utils::extract_initializable_landmark_id(const vislam::data::frame &frame_current,
                                                               const std::unordered_map<uint64_t, vislam::data::landmark> &database_landmark) {


    return std::vector<uint64_t>();
}

std::vector<vislam::Vec3_t> utils::triangulate_landmark_position(const std::vector<uint64_t> &initializable_landmark_id,
                                                                 const std::unordered_map<uint64_t, vislam::data::landmark> &database_landmark) {


    return std::vector<vislam::Vec3_t>();
}


