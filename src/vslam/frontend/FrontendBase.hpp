// 
// Created by ery on 2020/04/24.
//

#pragma once

#include "ThreadsafeContainer.hpp"
#include "opencv2/opencv.hpp"

namespace vslam::frontend {

class FrontendStatus {
 public:
  FrontendStatus() : dummy_(false) {}
  bool dummy_;

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
  //  virtual FrontendStatus FeedImage(const cv::Mat& image) = 0;

 protected:
  std::shared_ptr<data::ThreadsafeMapDatabase> map_database_;
};

}  // namespace vslam::frontend