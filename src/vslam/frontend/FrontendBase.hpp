// 
// Created by ery on 2020/04/24.
//

#pragma once

#include "Camera.hpp"
#include "Frame.hpp"
#include "Landmark.hpp"
#include "ThreadsafeContainer.hpp"
#include "opencv2/opencv.hpp"

namespace vslam::frontend {

class FrontendStatus {
 public:
  FrontendStatus() : dummy_(false) {}
  bool dummy_;

 private:
};

class FrontendInput {
 public:
  FrontendInput();
  FrontendInput(double timestamp,
                const cv::Mat& frame,
                const cv::Mat& mask,
                const std::unique_ptr<data::CameraModelBase>& camera_model);
  FrontendInput(const FrontendInput& frontend_input);

  FrontendInput& operator=(const FrontendInput& frontend_input);

  double timestamp_;
  cv::Mat frame_;
  cv::Mat mask_;
  std::unique_ptr<data::CameraModelBase> camera_model_ptr_;

 private:
};

/**
 * @brief FrontEndの大枠を定義
 * @details
 * ミッションは一連の入力画像からMapを作って、MapDatabaseを有意な情報で満たすこと。
 */

class FrontendBase {
 public:
  explicit FrontendBase(
      const std::shared_ptr<data::ThreadsafeMapDatabase>& map_database);
  virtual ~FrontendBase() = default;

  //  virtual FrontendStatus FeedImage(const cv::Mat& image) = 0;

 protected:
  std::shared_ptr<data::ThreadsafeMapDatabase> map_database_;
};

}  // namespace vslam::frontend