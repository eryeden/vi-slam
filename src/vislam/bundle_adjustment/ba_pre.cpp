//
// Created by ery on 2020/02/17.
//

#include "ba_pre.hpp"

vislam::ba::ba_observation::ba_observation() {
;
}


vislam::ba::ba_pre::ba_pre() {
    ;
}


/**
 * @brief BAで利用するFrameとLandmarkを、Databaseから選ぶ
 * @details
 * 抽出はDatabaseのIDベースで行って、データの内容にアクセスしたい場合はDatabaseにIDアクセスを行う。
 * そうするとBAで必要なヤコビアンや最適化対象の変数保存場所などは別のところ、別の構造体として保存したほうが良い？
 * こっちのやり方を考えたほうがいいかもしれない。
 * 処理の流れ：
 * 共通するランドマークのFrame IDをだす？
 * BA対象とするLandmarkの選び方が結構問題になりそう
 *
 * ## BA対象Landmarkの選び方
 * 複数のフレームに観測されているLandmarkをどの段階でBA対象とするかが問題になる。
 * また、どの段階でBA対象から外すか？も問題となるはず。
 * 普通に考えると次の案がある？
 * 案１：最低n frameにまたがって観測されたらBA対象にする
 * 案２：観測したときの視差がある値以上になったらBA対象にする（ラフに推定したカメラ位置が、ある程度離れていたらBA対象の特徴点にする）
 * OrbSLAMのやり方など調査する必要あり
 *
 * ## LandmarkのBA対象からの外し方
 * 案１：いままでBA対象だった特徴点を観測するFrameの数がn frame以下になった場合
 * 案２：観測特徴点間の最低視差がある程度以下になった場合
 * こちらもOrbSLAMでのやり方など調査する必要あり
 *
 * ## Landmarkの選び方の効率的な方法は？
 * おそらく、Landmark数は大きな値になって、この採用するかどうかをいちいち判定していると時間がかかってしまう気がする。
 * なので、対象Frameの最初と、最後に含まれる特徴点のみ、採用判断対象Landmarkにするなどの対応が必要になると思う。
 * また、OrbSLAMで使っているCo-visibility graphなど利用したほうが簡単に実装できるかもしれない。
 *
 */
void vislam::ba::ba_pre::select_frames_and_landmarks(const std::unordered_map<uint64_t, data::frame> &input_frame_database,
                                                     const std::unordered_map<uint64_t, data::landmark> &input_landmark_database,
                                                     uint64_t window_size,
                                                     uint64_t latest_frame_id,
                                                     std::vector<ba_observation> &selected_frame_database,
                                                     std::vector<uint64_t> &selected_landmark_database) {

    /**
     * @brief BA対象Landmarkの選び方：
     * @detals
     * 観測回数でフィルタすることにする。実装としては、観測回数とLandmarkIDを紐付けたMapを用意して、ここで一旦観測回数をカウントしたあとに、
     * 観測回数によるフィルタを行うとする。
     * 本当は、結果をキャッシュして、新たに観測されたフレームと、最も最後に観測されているフレームのみから更新する事ができるはず。
     */
     const uint64_t observation_counter_threshold = 3;
     std::unordered_map<uint64_t , uint64_t > landmark_observing_counter;
     for(const auto &[frame_id, frm]: input_frame_database){
         for(const auto landmark_id: frm.observingFeatureId){
             if(landmark_observing_counter.count(landmark_id) == 0){
                 landmark_observing_counter[landmark_id] = 1;
             }else{
                 landmark_observing_counter[landmark_id]++;
             }
         }
     }
     std::vector<uint64_t > selected_landmark_id(0); // ここに今回のBAで対象となるLandmarkのIDが入る。
     selected_landmark_id.reserve(landmark_observing_counter.size());
     for(const auto&[landmark_id, observation_counter]: landmark_observing_counter){
         if(observation_counter >= observation_counter_threshold){
             selected_landmark_id.emplace_back(landmark_id);
         }
     }
    //! このあと、対象フレームでの観測ランドマークIDとBA対象ランドマークIDの積集合をとるために、std::set_intersectionを実行するが、そのためにSortしておく。
    std::sort(selected_landmark_id.begin(), selected_landmark_id.end());
    //! 結果の出力
    selected_landmark_database = selected_landmark_id;

    /**
     * ba_observationを生成する
     */
    std::vector<ba_observation> selected_frame;
    selected_frame.reserve(input_frame_database.size());
    for(const auto &[frame_id, frm]: input_frame_database){
        std::vector<uint64_t> landmark_id_observed_by_this_frame(frm.observingFeatureId.begin(), frm.observingFeatureId.end());
        std::vector<uint64_t> ba_landmark_id_observed_by_this_frame(0);
        std::set_intersection(selected_landmark_id.begin(), selected_landmark_id.end(),
                              landmark_id_observed_by_this_frame.begin(), landmark_id_observed_by_this_frame.end(),
                 std::back_inserter(ba_landmark_id_observed_by_this_frame));

        ba_observation current_ba_observation;
        current_ba_observation.frame_id = frm.id;
        current_ba_observation.landmark_id = ba_landmark_id_observed_by_this_frame;

        selected_frame.emplace_back(current_ba_observation);
    }
    //! BA対象の特徴点IDを詰め込んだFrameDatabaseを出力する
    selected_frame_database = selected_frame;
}

void vislam::ba::ba_pre::fill_derivatives(const std::unordered_map<uint64_t, data::frame> &input_frame_database,
                                          const std::unordered_map<uint64_t, data::landmark> &input_landmark_database,
                                          std::vector<ba_observation> &selected_frame_database) {




}

Eigen::SparseMatrix<double>
vislam::ba::ba_pre::generate_jacobian(const std::vector<ba_observation> &selected_frame_database,
                                      const std::vector<uint64_t> &selected_landmark_database) {
    return Eigen::SparseMatrix<double>();
}

