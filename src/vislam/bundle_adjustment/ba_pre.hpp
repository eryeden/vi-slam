//
// Created by ery on 2020/02/17.
//

#pragma once

#include "camera.hpp"
#include "frame.hpp"
#include "landmark.hpp"
#include "type_defines.hpp"

namespace vislam::ba{

    /**
     * @brief フレームごとのヤコビアン計算に必要なものを保存する
     */
    struct ba_observation{
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        ba_observation();

        uint64_t frame_id; //! 対象にするFrameのID

        /**
         * @brief 今回のBA対象となっている特徴点のうち、このフレームで観測されているもののIDリスト。以下変数はここでリストアップされた特徴点IDについて計算された結果になる
         * @brief 並びの順序保証は landmark_idが、std::setなのでこれを利用すればOK。
         */
        std::set<uint64_t > landmark_id;
        eigen_allocated_unordered_map<uint64_t, Vec2_t> reprojection_error; //! 再投影誤差
        eigen_allocated_unordered_map<uint64_t, MatRC_t<2,3>> df_dp; //! der_F / der_p^W
        eigen_allocated_unordered_map<uint64_t, MatRC_t<2,3>> df_domega; //! der_F / der_omega
        eigen_allocated_unordered_map<uint64_t, MatRC_t<2, 3>>df_dt; //! der_F / der_t
    };

    /**
     * @brief 役割：パラメータの初期化以外のBA処理全体を実装するクラス
     * @details
     * となると、特に中間データも必要ない？Utilityクラスとしての実装になるかもしれない。
     * ## 入力と出力について
     * - 入力：frame_database, landmark_database, sliding_window size?
     */
    class ba_pre{

    public:

        ba_pre();

        /**
         * @brief
         */



    private:


    };

}

