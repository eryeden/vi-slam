//
// Created by ery on 2020/02/17.
//

#pragma once

#include "camera.hpp"
#include "frame.hpp"
#include "landmark.hpp"

#include "Eigen/Sparse"

namespace vislam::ba {

    /**
     * @brief Frameで観測されていて、BA対象のFeatureに対する、再投影誤差、各種偏微分値が記録される
     */
    struct ba_observation {

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW


        /**
         *
         */
        ba_observation();

        /**
         * 対象のFrame ID
         */
        uint64_t frame_id;

        /**
         * @brief BAで利用する特徴点のうち、このフレームで利用する特徴点IDを登録しておく
         */
        std::vector<uint64_t> landmark_id;
        eigen_allocated_unordered_map<uint64_t, Vec2_t> reprojection_error; //! 再投影誤差F
        eigen_allocated_unordered_map<uint64_t, MatRC_t<2, 3>> der_f_der_omega; //! Fの角速度omega偏微分
        eigen_allocated_unordered_map<uint64_t, MatRC_t<2, 3>> der_f_der_t; //! Fの移動量t偏微分
        eigen_allocated_unordered_map<uint64_t, MatRC_t<2, 3>> der_f_der_p; //! Fの特徴点位置偏微分

    };

    /**
     * @brief 役割：パラメータの初期化以外のBA処理全体を実装するクラス
     * @details
     * となると、特に中間データも必要ない？Utilityクラスとしての実装になるかもしれない。
     * ## 入力と出力について
     * - 入力：frame_database, landmark_database, sliding_window size?
     */
    class ba_pre {

        /**
         * @brief 今の所特になし
         */
        ba_pre();

        /**
         * @brief BAで利用するFrameとLandmarkを、Databaseから選ぶ
         * @details
         * 抽出はDatabaseのIDベースで行って、データの内容にアクセスしたい場合はDatabaseにIDアクセスを行う。
         * そうするとBAで必要なヤコビアンや最適化対象の変数保存場所などは別のところ、別の構造体として保存したほうが良い？
         * こっちのやり方を考えたほうがいいかもしれない
         * @param input_frame_database
         * @param input_landmark_database
         * @param window_size
         * @param output_observation_database
         * @param output_landmark_id_database
         */
        static void select_frames_and_landmarks(
                const std::unordered_map<uint64_t, data::frame> &input_frame_database,
                const std::unordered_map<uint64_t, data::landmark> &input_landmark_database,
                uint64_t window_size,
                uint64_t latest_frame_id,
                std::vector<ba_observation> &selected_frame_database,
                std::vector<uint64_t> &selected_landmark_database
        );

        /**
         * @brief 偏微分を計算、結果をselected_frame_databaseに保存する
         * @param input_frame_database
         * @param input_landmark_database
         * @param selected_frame_database
         */
        static void fill_derivatives(const std::unordered_map<uint64_t, data::frame> &input_frame_database,
                                     const std::unordered_map<uint64_t, data::landmark> &input_landmark_database,
                                     std::vector<ba_observation> &selected_frame_database
        );

        /**
         * @brief 偏微分の計算結果からJacobianを生成する
         * @param selected_frame_database
         * @param selected_landmark_database
         * @return
         */
        static Eigen::SparseMatrix<double> generate_jacobian(const std::vector<ba_observation> &selected_frame_database,
                                                             const std::vector<uint64_t> &selected_landmark_database);

    };

}

