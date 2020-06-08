//
// Created by ery on 2020/06/08.
//
#include <spdlog/spdlog.h>

#include "FeatureTrackerLucasKanade.hpp"

using namespace vslam;
using namespace vslam::feature;

FeatureTrackerLSSDLucasKanade::FeatureTrackerLSSDLucasKanade(
    int32_t frame_width,
    int32_t frame_height,
    int32_t klt_max_iteration,
    double klt_epsilon,
    int32_t klt_window_size,
    int32_t klt_max_level,
    double backtrack_distance_threshold)
    : timestamp_ns_(1), is_first_(true) {
  /**
   * @brief Setup calib
   * @details
   * basalt::opticalflowでは、単眼とStereo以上で利用するcamera calibration
   * データが異なる。
   * - mono : image resolutionのみ
   * - stereo : camera L to camera Rの姿勢, それぞれののカメラモデル, image
   * resolution
   * ということで、ここでは単眼、画像サイズと中に何もないカメラモデルのみ設定している。
   */
  camera_calibration_ = basalt::Calibration<double>();
  camera_calibration_.resolution = {Eigen::Vector2i(frame_width, frame_height)};
  camera_calibration_.intrinsics = {basalt::GenericCamera<double>()};

  /**
   * @brief Setup VIO config
   */
  vio_config_ = basalt::VioConfig();

  /**
   * @brief Setup opticalflow
   */
  optical_flow_ptr_ = basalt::OpticalFlowFactory::getOpticalFlow(
      vio_config_, camera_calibration_);
  optical_flow_ptr_->output_queue = &observations_queue_;
}

void FeatureTrackerLSSDLucasKanade::Track(
    FeaturePositionDatabase& feature_position,
    FeatureAgeDatabase& feature_age,
    const cv::Mat& prev_image,
    const cv::Mat& current_frame) {}

void FeatureTrackerLSSDLucasKanade::Track(const cv::Mat& current_frame) {
  /**
   * @brief basalt::imageの形式にOpenCV::Matを変換、OpticalFlowInputを生成
   */
  //@{
  basalt::OpticalFlowInput::Ptr data(new basalt::OpticalFlowInput);
  data->img_data.resize(1);
  data->img_data[0].img.reset(new basalt::ManagedImage<uint16_t>(
      current_frame.cols, current_frame.rows));
  const uint8_t* data_in = current_frame.ptr();
  uint16_t* data_out = data->img_data[0].img->ptr;
  size_t full_size = current_frame.cols * current_frame.rows;
  for (size_t i = 0; i < full_size; i++) {
    int val = data_in[i];
    val = val << 8;
    data_out[i] = val;
  }
  data->t_ns = timestamp_ns_++;
  //@}

  /**
   * @brief Feed data to optical flow
   */
  optical_flow_ptr_->input_queue.push(data);
  if (is_first_) {
    is_first_ = false;
    return;
  }

  /**
   * @brief Get data from optical flow
   * @details
   * optical flowの計算は別スレッドで実行されているので、
   * tbb concurrent queue経由で通信をする。処理が終わっていない場合、
   * queueにはなにも入っていいないので、内容が受信できるまでポーリングする。
   * tbbはARMでも動くらしいのでJetsonでの動作もとくに問題ではなさそう。
   *
   */
  basalt::OpticalFlowResult::Ptr res;
  EigenAllocatedUnorderedMap<database_index_t, Vec2_t> key_points;
  while (true) {
    if (optical_flow_ptr_->output_queue != nullptr) {
      optical_flow_ptr_->output_queue->pop(res);
      if (res.get()) break;
    }
  }
  const auto& observation = res->observations[0];
  for (const auto& [id, obs] : observation) {
    Eigen::MatrixXf transformed_patch =
        obs.linear() * optical_flow_ptr_->patch_coord;
    transformed_patch.colwise() += obs.translation();
    key_points[id] = {obs.translation().x(), obs.translation().y()};
  }

  /**
   * @brief Visuallize
   */
  cv::Mat vis;
  cv::cvtColor(current_frame, vis, CV_GRAY2BGR);
  for (const auto& [id, pos] : key_points) {
    cv::circle(vis, cv::Point(pos[0], pos[1]), 2, cv::Scalar(0, 255, 0), 1);
  }
  cv::imshow("LSSD", vis);
  cv::waitKey(1);
}