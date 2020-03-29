//
// Created by ery on 2020/02/17.
//

#include <iostream>
#include "ba_pre.hpp"
#include "geometry.hpp"

#include <Eigen/SparseCholesky>
#include <Eigen/SparseQR>
#include <Eigen/Geometry>

#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

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
 * 案３：単に初期化に成功しており、Outlier判定されていないLandmarkをBA対象とする <=ここではコレを実装！！！
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
void vislam::ba::ba_pre::select_frames_and_landmarks(
    const std::unordered_map<uint64_t, data::frame> &input_frame_database,
    const std::unordered_map<uint64_t, data::landmark> &input_landmark_database,
    uint64_t window_size,
    uint64_t latest_frame_id,
    std::vector<ba_observation> &selected_observation_database,
    std::vector<uint64_t> &selected_landmark_database) {

//    /**
//     * @brief BA対象Landmarkの選び方：
//     * @detals
//     * 観測回数でフィルタすることにする。実装としては、観測回数とLandmarkIDを紐付けたMapを用意して、ここで一旦観測回数をカウントしたあとに、
//     * 観測回数によるフィルタを行うとする。
//     * 本当は、結果をキャッシュして、新たに観測されたフレームと、最も最後に観測されているフレームのみから更新する事ができるはず。
//     */
//     const uint64_t observation_counter_threshold = 3;
//     std::unordered_map<uint64_t , uint64_t > landmark_observing_counter;
//     for(const auto &[frame_id, frm]: input_frame_database){
//         for(const auto landmark_id: frm.observingFeatureId){
//             if(landmark_observing_counter.count(landmark_id) == 0){
//                 landmark_observing_counter[landmark_id] = 1;
//             }else{
//                 landmark_observing_counter[landmark_id]++;
//             }
//         }
//     }
//     std::vector<uint64_t > selected_landmark_id(0); // ここに今回のBAで対象となるLandmarkのIDが入る。
//     selected_landmark_id.reserve(landmark_observing_counter.size());
//     for(const auto&[landmark_id, observation_counter]: landmark_observing_counter){
//         if(observation_counter >= observation_counter_threshold){
//             selected_landmark_id.emplace_back(landmark_id);
//         }
//     }

  /**
   * @brief BA対象のLandmarkを選択する
   * @details
   * ここでは、以下の条件のLandmarkを選択するとした。
   * - Outlierではない
   * - 初期化に成功している
   * - BA対象のFrameに観測されている
   */
  std::vector<uint64_t> selected_landmark_id(0); // ここに今回のBAで対象となるLandmarkのIDが入る。
  for (const auto &[frame_id, frm]: input_frame_database) {
    for (const auto landmark_id: frm.observingFeatureId) {
      const auto &current_landmark = input_landmark_database.at(landmark_id);
      if ((!current_landmark.isOutlier) && current_landmark.isInitialized) {
        selected_landmark_id.emplace_back(landmark_id);
      }
    }
  }
  //! このあと、対象フレームでの観測ランドマークIDとBA対象ランドマークIDの積集合をとるために、std::set_intersectionを実行するが、そのためにSortしておく。
  std::sort(selected_landmark_id.begin(), selected_landmark_id.end());
  //! 重複IDを削除する
  selected_landmark_id.erase(std::unique(selected_landmark_id.begin(), selected_landmark_id.end()),
                             selected_landmark_id.end());
  //! 結果の出力

  selected_landmark_database = selected_landmark_id;

  /**
   * ba_observationを生成する
   */
  std::vector<ba_observation> selected_frame;
  selected_frame.reserve(input_frame_database.size());

  std::vector<uint64_t> sorted_frame_ids;
  for (const auto &[frame_id, frame]:input_frame_database) {
    sorted_frame_ids.emplace_back(frame_id);
  }
  std::sort(sorted_frame_ids.begin(), sorted_frame_ids.end());
  for (const auto frame_id : sorted_frame_ids) {
    const auto &frm = input_frame_database.at(frame_id);
    std::vector<uint64_t> landmark_id_observed_by_this_frame(frm.observingFeatureId.begin(),
                                                             frm.observingFeatureId.end());
    std::vector<uint64_t> ba_landmark_id_observed_by_this_frame(0);
    std::set_intersection(selected_landmark_id.begin(), selected_landmark_id.end(),
                          landmark_id_observed_by_this_frame.begin(), landmark_id_observed_by_this_frame.end(),
                          std::back_inserter(ba_landmark_id_observed_by_this_frame));

    ba_observation current_ba_observation;
    current_ba_observation.frame_id = frm.id;
    current_ba_observation.landmark_id = ba_landmark_id_observed_by_this_frame;

    selected_frame.emplace_back(current_ba_observation);
  }
//! Range based forでunordered_mapのLoopを回すと順番はめちゃくちゃになるので …
//  for (const auto &[frame_id, frm]: input_frame_database) {
//    std::vector<uint64_t> landmark_id_observed_by_this_frame(frm.observingFeatureId.begin(),
//                                                             frm.observingFeatureId.end());
//    std::vector<uint64_t> ba_landmark_id_observed_by_this_frame(0);
//    std::set_intersection(selected_landmark_id.begin(), selected_landmark_id.end(),
//                          landmark_id_observed_by_this_frame.begin(), landmark_id_observed_by_this_frame.end(),
//                          std::back_inserter(ba_landmark_id_observed_by_this_frame));
//
//    ba_observation current_ba_observation;
//    current_ba_observation.frame_id = frm.id;
//    current_ba_observation.landmark_id = ba_landmark_id_observed_by_this_frame;
//
//    selected_frame.emplace_back(current_ba_observation);
//  }
  //! BA対象の特徴点IDを詰め込んだFrameDatabaseを出力する
  selected_observation_database = selected_frame;
}

/**
 * @brief ba_observationに各Frameに写ったLandmarkの偏微分を詰めていく（der_omega, der_t, der_p）
 * @param input_frame_database
 * @param input_landmark_database
 * @param selected_frame_database
 */
void vislam::ba::ba_pre::fill_derivatives(const std::unordered_map<uint64_t, data::frame> &input_frame_database,
                                          const std::unordered_map<uint64_t, data::landmark> &input_landmark_database,
                                          std::vector<ba_observation> &selected_frame_database) {
//    std::cout << "#######################" << std::endl;
  for (auto &ba_obs : selected_frame_database) {
    uint64_t frame_id = ba_obs.frame_id;
    Vec3_t frame_position = input_frame_database.at(frame_id).cameraPosition;
    Mat33_t frame_attitude = input_frame_database.at(frame_id).cameraAttitude.normalized().toRotationMatrix();
    Mat33_t camera_intrinsic = input_frame_database.at(frame_id).cameraParameter.get_intrinsic_matrix();

//        std::cout << "Frame ID: " << frame_id << std::endl;
//        std::cout << "Frame Position: " << frame_position << std::endl;
//        std::cout << "Frame Attitude: " << frame_attitude << std::endl;


    for (const auto landmark_id : ba_obs.landmark_id) {
      Vec2_t landmark_position_in_device = input_frame_database.at(frame_id).observingFeaturePointInDevice.at(
          landmark_id);
      Vec3_t landmark_position_in_world = input_landmark_database.at(landmark_id).positionInWorld;

      ba_obs.reprojection_error[landmark_id] = geometry::utility::get_reprojection_error(
          landmark_position_in_device,
          landmark_position_in_world,
          frame_attitude,
          frame_position,
          camera_intrinsic);
      ba_obs.der_f_der_omega[landmark_id] = geometry::utility::get_der_F_der_omega(
          landmark_position_in_world,
          frame_attitude,
          frame_position,
          camera_intrinsic);
      ba_obs.der_f_der_t[landmark_id] = geometry::utility::get_der_F_der_t(
          landmark_position_in_world,
          frame_attitude,
          frame_position,
          camera_intrinsic);
      ba_obs.der_f_der_p[landmark_id] = geometry::utility::get_der_F_der_p(
          landmark_position_in_world,
          frame_attitude,
          frame_position,
          camera_intrinsic);
    }
  }
}

/**
 * @brief 選択したFrameとLandmarkの偏微分関係が全て計算されているはずなので配置していく
 * @param selected_frame_database
 * @param selected_landmark_database
 * @return
 */
Eigen::SparseMatrix<double> vislam::ba::ba_pre::generate_jacobian(
    const std::vector<ba_observation> &selected_frame_database,
    const std::vector<uint64_t> &selected_landmark_database) {

  /**
   * @brief jacobianのサイズを計算する
   */
  int64_t num_raw_jacobian = 0, num_col_jacobian = 0;
  //! 縦サイズ：全フレームの観測ランドマーク数の合計
  for (const auto &ba_obs: selected_frame_database) {
    num_raw_jacobian += ba_obs.landmark_id.size() * 2;
  }
  num_col_jacobian = selected_frame_database.size() * 6 +
      selected_landmark_database.size() * 3; //! 横サイズ：BA対象Frame数＋BA対象Landmark数

  /**
   * @brief BAに利用するLandmark idとそれが配置される順番（std::vectorでのインデックス）が必要になるので予め作成しておく
   */
//    std::cout << "Landmark Index: " << std::endl;
  std::unordered_map<uint64_t, uint64_t> landmark_id_to_array_index_map;
  for (size_t landmark_array_index = 0;
       landmark_array_index < selected_landmark_database.size(); landmark_array_index++) {
//        std::cout << landmark_array_index << ", " << selected_landmark_database[landmark_array_index] << std::endl;
    landmark_id_to_array_index_map[selected_landmark_database[landmark_array_index]] = landmark_array_index;
  }

  /**
   * @brief 粗行列としてJacobianを生成する
   * @note
   * インデックスの計算が結構間違っていたので注意しようね
   */
  Eigen::SparseMatrix<double> jacobian(num_raw_jacobian, num_col_jacobian);

  int64_t raw_index = 0;
  for (size_t frame_index = 0; frame_index < selected_frame_database.size(); frame_index++) {
//        std::cout << frame_index << "/" << selected_frame_database.size() << std::endl;

    for (size_t inframe_landmark_index = 0; inframe_landmark_index <
        selected_frame_database[frame_index].landmark_id.size(); inframe_landmark_index++) {

//            std::cout << inframe_landmark_index << "/" << selected_frame_database[frame_index].landmark_id.size() << std::endl;


      int64_t col_index = 0;
      const auto current_landmark_id = selected_frame_database[frame_index].landmark_id[inframe_landmark_index];
      /**
       * @brief insert der_F_der_omega
       */
//            col_index = inframe_landmark_index*6;
      col_index = frame_index * 6;
      jacobian.insert(raw_index, col_index) = selected_frame_database[frame_index].der_f_der_omega.at(
          current_landmark_id)(0, 0);
      jacobian.insert(raw_index, col_index + 1) = selected_frame_database[frame_index].der_f_der_omega.at(
          current_landmark_id)(0, 1);
      jacobian.insert(raw_index, col_index + 2) = selected_frame_database[frame_index].der_f_der_omega.at(
          current_landmark_id)(0, 2);
      jacobian.insert(raw_index + 1, col_index) = selected_frame_database[frame_index].der_f_der_omega.at(
          current_landmark_id)(1, 0);
      jacobian.insert(raw_index + 1, col_index + 1) = selected_frame_database[frame_index].der_f_der_omega.at(
          current_landmark_id)(1, 1);
      jacobian.insert(raw_index + 1, col_index + 2) = selected_frame_database[frame_index].der_f_der_omega.at(
          current_landmark_id)(1, 2);
//            jacobian.insert(raw_index, col_index) =  selected_frame_database[frame_index].der_f_der_omega.at(inframe_landmark_index)(0,0);
//            jacobian.insert(raw_index, col_index+1) =  selected_frame_database[frame_index].der_f_der_omega.at(inframe_landmark_index)(0,1);
//            jacobian.insert(raw_index, col_index+2) =  selected_frame_database[frame_index].der_f_der_omega.at(inframe_landmark_index)(0,2);
//            jacobian.insert(raw_index+1, col_index) =  selected_frame_database[frame_index].der_f_der_omega.at(inframe_landmark_index)(1,0);
//            jacobian.insert(raw_index+1, col_index+1) =  selected_frame_database[frame_index].der_f_der_omega.at(inframe_landmark_index)(1,1);
//            jacobian.insert(raw_index+1, col_index+2) =  selected_frame_database[frame_index].der_f_der_omega.at(inframe_landmark_index)(1,2);

      /**
       * @brief insert der_F_der_t
       */
//            col_index = inframe_landmark_index*6+3;
      col_index = frame_index * 6 + 3;
//            jacobian.insert(raw_index, col_index) =  selected_frame_database[frame_index].der_f_der_t.at(inframe_landmark_index)(0,0);
//            jacobian.insert(raw_index, col_index+1) =  selected_frame_database[frame_index].der_f_der_t.at(inframe_landmark_index)(0,1);
//            jacobian.insert(raw_index, col_index+2) =  selected_frame_database[frame_index].der_f_der_t.at(inframe_landmark_index)(0,2);
//            jacobian.insert(raw_index+1, col_index) =  selected_frame_database[frame_index].der_f_der_t.at(inframe_landmark_index)(1,0);
//            jacobian.insert(raw_index+1, col_index+1) =  selected_frame_database[frame_index].der_f_der_t.at(inframe_landmark_index)(1,1);
//            jacobian.insert(raw_index+1, col_index+2) =  selected_frame_database[frame_index].der_f_der_t.at(inframe_landmark_index)(1,2);
      jacobian.insert(raw_index, col_index) = selected_frame_database[frame_index].der_f_der_t.at(
          current_landmark_id)(0, 0);
      jacobian.insert(raw_index, col_index + 1) = selected_frame_database[frame_index].der_f_der_t.at(
          current_landmark_id)(0, 1);
      jacobian.insert(raw_index, col_index + 2) = selected_frame_database[frame_index].der_f_der_t.at(
          current_landmark_id)(0, 2);
      jacobian.insert(raw_index + 1, col_index) = selected_frame_database[frame_index].der_f_der_t.at(
          current_landmark_id)(1, 0);
      jacobian.insert(raw_index + 1, col_index + 1) = selected_frame_database[frame_index].der_f_der_t.at(
          current_landmark_id)(1, 1);
      jacobian.insert(raw_index + 1, col_index + 2) = selected_frame_database[frame_index].der_f_der_t.at(
          current_landmark_id)(1, 2);

      /**
       * @brief insert der_F_der_p
       */
//            col_index = (selected_frame_database[frame_index].landmark_id.size()-1)*6+3 + 1
//                    + 3*landmark_id_to_array_index_map[selected_frame_database[frame_index].landmark_id[inframe_landmark_index]]; // p_alpha^Wの並び順Indexを取得する
      col_index = selected_frame_database.size() * 6 +
          3 * landmark_id_to_array_index_map[current_landmark_id]; // p_alpha^Wの並び順Indexを取得する

//            col_index = selected_frame_database.size()*6 + landmark_id_to_array_index_map[current_landmark_id]; // p_alpha^Wの並び順Indexを取得する

//            jacobian.insert(raw_index, col_index) =  selected_frame_database[frame_index].der_f_der_p.at(inframe_landmark_index)(0,0);
//            jacobian.insert(raw_index, col_index+1) =  selected_frame_database[frame_index].der_f_der_p.at(inframe_landmark_index)(0,1);
//            jacobian.insert(raw_index, col_index+2) =  selected_frame_database[frame_index].der_f_der_p.at(inframe_landmark_index)(0,2);
//            jacobian.insert(raw_index+1, col_index) =  selected_frame_database[frame_index].der_f_der_p.at(inframe_landmark_index)(1,0);
//            jacobian.insert(raw_index+1, col_index+1) =  selected_frame_database[frame_index].der_f_der_p.at(inframe_landmark_index)(1,1);
//            jacobian.insert(raw_index+1, col_index+2) =  selected_frame_database[frame_index].der_f_der_p.at(inframe_landmark_index)(1,2);

      jacobian.insert(raw_index, col_index) = selected_frame_database[frame_index].der_f_der_p.at(
          current_landmark_id)(0, 0);
      jacobian.insert(raw_index, col_index + 1) = selected_frame_database[frame_index].der_f_der_p.at(
          current_landmark_id)(0, 1);
      jacobian.insert(raw_index, col_index + 2) = selected_frame_database[frame_index].der_f_der_p.at(
          current_landmark_id)(0, 2);
      jacobian.insert(raw_index + 1, col_index) = selected_frame_database[frame_index].der_f_der_p.at(
          current_landmark_id)(1, 0);
      jacobian.insert(raw_index + 1, col_index + 1) = selected_frame_database[frame_index].der_f_der_p.at(
          current_landmark_id)(1, 1);
      jacobian.insert(raw_index + 1, col_index + 2) = selected_frame_database[frame_index].der_f_der_p.at(
          current_landmark_id)(1, 2);

      raw_index += 2; //! F_alpha_kappaの要素数は２なので、２個シフトする
    }
  }

  jacobian.finalize();

  return jacobian;
}

Eigen::VectorXd vislam::ba::ba_pre::reprojection_error_vector(const std::vector<ba_observation> &ba_observation_database,
                                                              const Eigen::SparseMatrix<double> &jacobian) {
  Eigen::VectorXd residuals(jacobian.rows());
  int64_t raw_index = 0;
  for (size_t frame_index = 0; frame_index < ba_observation_database.size(); frame_index++) {
    for (size_t inframe_landmark_index = 0; inframe_landmark_index <
        ba_observation_database[frame_index].landmark_id.size(); inframe_landmark_index++) {
      const auto current_landmark_id = ba_observation_database[frame_index].landmark_id[inframe_landmark_index];
      residuals(raw_index, 0) = ba_observation_database[frame_index].reprojection_error.at(
          current_landmark_id)[0];
      residuals(raw_index + 1, 0) = ba_observation_database[frame_index].reprojection_error.at(
          current_landmark_id)[1];
      raw_index += 2;
    }
  }
  return residuals;
}

Eigen::VectorXd vislam::ba::ba_pre::generate_gradient(const std::vector<ba_observation> &ba_observation_database,
                                                      const Eigen::SparseMatrix<double> &jacobian) {
  Eigen::VectorXd residuals(jacobian.rows());

  int64_t raw_index = 0;
  for (size_t frame_index = 0; frame_index < ba_observation_database.size(); frame_index++) {
    for (size_t inframe_landmark_index = 0; inframe_landmark_index <
        ba_observation_database[frame_index].landmark_id.size(); inframe_landmark_index++) {
      const auto current_landmark_id = ba_observation_database[frame_index].landmark_id[inframe_landmark_index];
      residuals(raw_index, 0) = ba_observation_database[frame_index].reprojection_error.at(
          current_landmark_id)[0];
      residuals(raw_index + 1, 0) = ba_observation_database[frame_index].reprojection_error.at(
          current_landmark_id)[1];
      raw_index += 2;
    }
  }
  return jacobian.transpose() * residuals;
}

void vislam::ba::ba_pre::do_the_ba(const std::unordered_map<uint64_t, data::frame> &input_frame_database,
                                   const std::unordered_map<uint64_t, data::landmark> &input_landmark_database,
                                   std::unordered_map<uint64_t, data::frame> &output_frame_database,
                                   std::unordered_map<uint64_t, data::landmark> &output_landmark_database) {

  /**
   * @brief 描画用の準備
   */
  //@{
  cv::Matx33d K;
  K << 458.654, 0.0000000000000000e+00, 367.215,
      0.0000000000000000e+00, 457.296, 248.375,
      0.0000000000000000e+00, 0.0000000000000000e+00, 1.0000000000000000e+00;
  cv::viz::Viz3d myWindow("Coordinate Frame");
  myWindow.showWidget("Coordinate Widget", cv::viz::WCoordinateSystem());
  //@}


  //! Landmarkの観測関係、Jacobianの元になる偏微分値など入れておく
  std::vector<vislam::ba::ba_observation> observation_database;
  //! 初期化対象のLandmark IDを入れておく
  std::vector<uint64_t> selected_landmark_id;

  //! BA対象のLandmarkを選択、BA情報をまとめるba_observationを生成する
  vislam::ba::ba_pre::select_frames_and_landmarks(
      input_frame_database,
      input_landmark_database,
      //! not used
      10,
      //! not used
      1,
      observation_database,
      selected_landmark_id);

  //! 選択されたFrameとLandmarkの最適化用変数を用意する
  //@{
  std::unordered_map<uint64_t, data::frame> opt_frame_database;
  std::unordered_map<uint64_t, data::landmark> opt_landmark_database;
  for (const auto &observation : observation_database) {
    opt_frame_database[observation.frame_id] = input_frame_database.at(observation.frame_id);
  }
  for (const auto &landmark_id : selected_landmark_id) {
    opt_landmark_database[landmark_id] = input_landmark_database.at(landmark_id);
  }
  //@}

  /**
   * @brief Do the BA. ひとまず、１００回くらい計算してみる
   */
  for (int i = 0; i < 100; i++) {
    //! 各Frameに観測されいているLandmarkの偏微分計算を実施、変数に偏微分結果を満たす
    vislam::ba::ba_pre::fill_derivatives(
        opt_frame_database,
        opt_landmark_database,
        observation_database);

    //! 平均の再投影誤差を計算する
    double mean_reprojection_error = 0;
    double number_of_obervation = 0;
    for (const auto &observation : observation_database) {
      for (const auto &[id, f]:observation.reprojection_error) {
        mean_reprojection_error += f.norm();
//                mean_reprojection_error += f[1]*f[1];
        number_of_obervation += 1;
//        std::cout << "Idx : " << number_of_obervation << ", " << f.norm() << std::endl;
      }
    }
    mean_reprojection_error = mean_reprojection_error / number_of_obervation;
    std::cout << "Idx : " << i << ", " << mean_reprojection_error << std::endl;

    //! Jacobianを計算する。満たした偏微分結果をSparseMatrixに代入する
    Eigen::SparseMatrix<double> j = vislam::ba::ba_pre::generate_jacobian(
        observation_database,
        selected_landmark_id);



    //! jacobianをFrame由来の成分JcとLandmark由来の成分Jpに分解する
    //@{
    Eigen::SparseMatrix<double> jc = j.block(0, 0, j.rows(), input_frame_database.size() * 6);
//    jc.coeffRef()
    //カメラのマスク
    int32_t num_mask_camera = 1;
    uint64_t mask_row_index = 0;
    for (int32_t mask_camera_index = 0; mask_camera_index < num_mask_camera; mask_camera_index++) {
      const auto &mask_observation = observation_database[mask_camera_index];
      for (int32_t mask_landmark_index = 0; mask_landmark_index < mask_observation.landmark_id.size();
           mask_landmark_index++) {
        int32_t mask_col_index = mask_camera_index * 6;
        jc.coeffRef(mask_row_index, mask_col_index) = 0;
        jc.coeffRef(mask_row_index, mask_col_index + 1) = 0;
        jc.coeffRef(mask_row_index, mask_col_index + 2) = 0;
        jc.coeffRef(mask_row_index, mask_col_index + 3) = 0;
        jc.coeffRef(mask_row_index, mask_col_index + 4) = 0;
        jc.coeffRef(mask_row_index, mask_col_index + 5) = 0;

        jc.coeffRef(mask_row_index + 1, mask_col_index) = 0;
        jc.coeffRef(mask_row_index + 1, mask_col_index + 1) = 0;
        jc.coeffRef(mask_row_index + 1, mask_col_index + 2) = 0;
        jc.coeffRef(mask_row_index + 1, mask_col_index + 3) = 0;
        jc.coeffRef(mask_row_index + 1, mask_col_index + 4) = 0;
        jc.coeffRef(mask_row_index + 1, mask_col_index + 5) = 0;

        mask_row_index += 2;
      }
    }

    for (int32_t mask_camera_index = 1; mask_camera_index < 2; mask_camera_index++) {
      const auto &mask_observation = observation_database[mask_camera_index];
      for (int32_t mask_landmark_index = 0; mask_landmark_index < mask_observation.landmark_id.size();
           mask_landmark_index++) {
        int32_t mask_col_index = mask_camera_index * 6;
        jc.coeffRef(mask_row_index, mask_col_index) = 0;
//        jc.coeffRef(mask_row_index, mask_col_index+1) = 0;
//        jc.coeffRef(mask_row_index, mask_col_index+2) = 0;
//        jc.coeffRef(mask_row_index, mask_col_index+3) = 0;
//        jc.coeffRef(mask_row_index, mask_col_index+4) = 0;
//        jc.coeffRef(mask_row_index, mask_col_index+5) = 0;

        jc.coeffRef(mask_row_index + 1, mask_col_index) = 0;
//        jc.coeffRef(mask_row_index+1, mask_col_index+1) = 0;
//        jc.coeffRef(mask_row_index+1, mask_col_index+2) = 0;
//        jc.coeffRef(mask_row_index+1, mask_col_index+3) = 0;
//        jc.coeffRef(mask_row_index+1, mask_col_index+4) = 0;
//        jc.coeffRef(mask_row_index+1, mask_col_index+5) = 0;
        mask_row_index += 2;
      }
    }
//    Eigen::SparseMatrix<double> jc(j.rows(), input_frame_database.size() * 6);
    Eigen::SparseMatrix<double> jp = j.block(0, input_frame_database.size() * 6, j.rows(),
                                             selected_landmark_id.size() * 3);
    //@}





    /**
 * @brief ヘシアンを近似し求めるが、ブロックごとに求める
 * @details
 * H = [Hcc Hcp / Hcp^T Hpp]として、Hcc, Hcp, Hppを計算する。
 * 全部計算するならば、これになる: Eigen::SparseMatrix<double> h = j.transpose() * j;
 */
    //@{
    Eigen::SparseMatrix<double> hcc, hcp, hpp;
    hcc = jc.transpose() * jc;
    hcp = jc.transpose() * jp;
    hpp = jp.transpose() * jp;

    for (int64_t si = 0; si < hcc.rows(); si++) {
      hcc.coeffRef(si, si) = hcc.coeffRef(si, si) + 10;
    }
    for (int64_t si = 0; si < hpp.rows(); si++) {
      hpp.coeffRef(si, si) = hpp.coeffRef(si, si) + 10;
    }

    //@}


    /**
     * @brief gradientを計算する。
     * @details
     * gもFrame成分とLandmark成分に分解して計算する。
     * g = [gc / gp]
     */
    //@{
    Eigen::VectorXd gc, gp;

    Eigen::VectorXd e = vislam::ba::ba_pre::reprojection_error_vector(observation_database, j);
    gc = jc.transpose() * e;
    gp = jp.transpose() * e;

//    Eigen::VectorXd g = vislam::ba::ba_pre::generate_gradient(observation_database, j);
//    gc = g.segment(0, input_frame_database.size() * 6);
//    gp = g.segment(input_frame_database.size() * 6, selected_landmark_id.size() * 3);
    //@}


    /**
     * @brief 補正量の計算をする
     */
    //@{
    //! hppの逆行列を計算する。hppは3x3の対称行列から構成されるので、低コストに逆行列をそのまま求められる。
    Eigen::SparseMatrix<double> inverse_hpp(hpp.rows(), hpp.cols());
    for (int64_t block_index = 0; block_index < hpp.rows(); block_index += 3) {
      vislam::Mat33_t tmp_hpp(hpp.block(block_index, block_index, 3, 3));
      vislam::Mat33_t inverse_tmp_hpp = tmp_hpp.inverse();
      for (int insert_index = 0; insert_index < 3; insert_index++) {
        inverse_hpp.insert(block_index + insert_index, block_index) = inverse_tmp_hpp(insert_index, 0);
        inverse_hpp.insert(block_index + insert_index, block_index + 1) = inverse_tmp_hpp(insert_index, 1);
        inverse_hpp.insert(block_index + insert_index, block_index + 2) = inverse_tmp_hpp(insert_index, 2);
      }
    }
    inverse_hpp.finalize();

    //! S = hcc - hcp inv_hpp hcp^T の計算
    Eigen::SparseMatrix<double> tmpS = hcp * inverse_hpp * hcp.transpose();
    Eigen::SparseMatrix<double> S = hcc - tmpS;
    vislam::MatX_t dense_s(S);// = hcc - tmpS;
    Eigen::VectorXd z = -gc + hcp * inverse_hpp * gp;

    Eigen::VectorXd delta_c, delta_p;
    //! S delta_c = -g_c + hcp inv_hpp g_pを計算する
//    Eigen::SparseLU<Eigen::SparseMatrix<double>> solver;
    Eigen::SimplicialLLT<Eigen::SparseMatrix<double>> solver;
//    Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> solver;
    solver.compute(S);
    delta_c = solver.solve(z);
//    delta_c = dense_s.inverse() * z;
    //! delta_p = inv_hpp * (-gp - hcp^T delta_c)を計算する
    delta_p = inverse_hpp * (-gp - hcp.transpose() * delta_c);
    //@}

    /**
     * @brief 変数の補正を行う
     */
    //! Frame関係の位置、姿勢の補正
    //! 最初のFrameの位置を基準にするため、その位置は動かさない。
    //! なので in_ba_frame_indexは1から始まる
    for (size_t in_ba_frame_index = 1; in_ba_frame_index < observation_database.size(); in_ba_frame_index++) {
      uint64_t frame_id = observation_database[in_ba_frame_index].frame_id;
      auto &frame = opt_frame_database[frame_id];

      //! 補正量の取得
      vislam::Vec3_t delta_omega = delta_c.segment(in_ba_frame_index * 6, 3);
      vislam::Vec3_t delta_translation = delta_c.segment(in_ba_frame_index * 6 + 3, 3);

//      vislam::Vec3_t delta_omega = delta_c.segment(in_ba_frame_index * 6+3, 3);
//      vislam::Vec3_t delta_translation = delta_c.segment(in_ba_frame_index * 6 , 3);

      //! 補正の実行
      //! 並行移動

      //! 回転の補正
      vislam::Mat33_t frame_rotation = frame.cameraAttitude.toRotationMatrix();
      vislam::Mat33_t delta_rotation = Eigen::AngleAxisd(delta_omega.norm(),
                                                         delta_omega.normalized()).toRotationMatrix();

      if (in_ba_frame_index == 1) {
        //frame.cameraPosition[0] = 1;
        frame.cameraPosition[1] += delta_translation[1];
        frame.cameraPosition[2] += delta_translation[2];

//        frame.cameraPosition += delta_translation;
        frame.cameraAttitude = vislam::Quat_t((delta_rotation * frame_rotation)).normalized();
      } else {
        frame.cameraPosition += delta_translation;
        frame.cameraAttitude = vislam::Quat_t((delta_rotation * frame_rotation)).normalized();
//        frame.cameraAttitude = vislam::Quat_t(frame_rotation*delta_rotation);
      }
//      frame.cameraPosition += delta_translation;
//      frame.cameraAttitude = vislam::Quat_t(delta_rotation * frame_rotation);
    }




    //! Landmark位置の補正
    for (size_t in_ba_landmark_index = 0;
         in_ba_landmark_index < selected_landmark_id.size(); in_ba_landmark_index++) {
      uint64_t landmark_id = selected_landmark_id[in_ba_landmark_index];
      auto &landmark = opt_landmark_database[landmark_id];

      //! 補正量の取得
      vislam::Vec3_t delta_landmark_translation = delta_p.segment(in_ba_landmark_index * 3, 3);

      //! 補正の実行
      landmark.positionInWorld += delta_landmark_translation;
    }





//        //! 表示する
    cv::Mat cv_hcc, cv_hcp, cv_hpp, cv_s, cv_j;
    Eigen::MatrixXd dense_hcc(hcc), dense_hcp(hcp), dense_hpp(hpp), dense_j(j.transpose());//, dense_s(S);
    cv::eigen2cv(dense_hcc, cv_hcc);
    cv::eigen2cv(dense_hcp, cv_hcp);
    cv::eigen2cv(dense_hpp, cv_hpp);
    cv::eigen2cv(dense_s, cv_s);
    cv::eigen2cv(dense_j, cv_j);
    cv::threshold(cv_hcc, cv_hcc, 0, 255, CV_THRESH_BINARY);
    cv::threshold(cv_hcp, cv_hcp, 0, 255, CV_THRESH_BINARY);
    cv::threshold(cv_hpp, cv_hpp, 0, 255, CV_THRESH_BINARY);
    cv::threshold(cv_s, cv_s, 0, 255, CV_THRESH_BINARY);
//
    cv::imshow("Hcc", cv_hcc);
    cv::imshow("Hcp", cv_hcp);
    cv::imshow("Hpp", cv_hpp);
    cv::imshow("S", cv_s);
    cv::imshow("J", cv_j);

//
//
//        cv::imwrite("/home/ery/hcc.png", cv_hcc);
//        cv::imwrite("/home/ery/hcp.png", cv_hcp);
//        cv::imwrite("/home/ery/hpp.png", cv_hpp);
//        cv::imwrite("/home/ery/S.png", cv_s);
//


    /**
     * @brief カメラ位置、Landmark位置を描画
     */
    // 特徴点位置を描画
    std::vector<cv::Point3d> pointCloud;
    for (auto &[landmark_id, localized_landmark] : opt_landmark_database) {
      pointCloud.emplace_back(
          cv::Point3d(localized_landmark.positionInWorld[0],
                      localized_landmark.positionInWorld[1],
                      localized_landmark.positionInWorld[2]));
    }
    cv::viz::WCloud cloud(pointCloud);
    myWindow.showWidget("CLOUD", cloud);

    for (const auto&[frame_id, frame] : opt_frame_database) {
      cv::viz::WCameraPosition tmp_wcamera(K, 1.0, cv::viz::Color::magenta());
      cv::Mat tmp_camera_attitude;
      cv::eigen2cv(frame.cameraAttitude.toRotationMatrix(), tmp_camera_attitude);
      cv::Affine3d tmp_cam_pose(tmp_camera_attitude,
                                cv::Vec3f(frame.cameraPosition[0], frame.cameraPosition[1], frame.cameraPosition[2]));
      myWindow.showWidget(std::to_string(frame_id), tmp_wcamera, tmp_cam_pose);
    }

//    myWindow.spin();
    myWindow.spinOnce(100);
//    cv::waitKey(0);


  }

  output_frame_database = opt_frame_database;
  output_landmark_database = opt_landmark_database;

}
