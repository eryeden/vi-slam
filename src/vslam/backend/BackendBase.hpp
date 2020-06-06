//
// Created by ery on 2020/04/24.
//

#pragma once

#include "ThreadsafeContainer.hpp"

namespace vslam::backend {

/**
 * @brief Backendの状態
 */
enum BackendState {
  BootStrap /*初期化状態*/,
  Nominal /*通常*/
};

/**
 * @brief Backendの大枠を準備しておく
 * @details
 * Backendの仕事は、Frontendがトラッキングしている特徴点をもとに、
 * 特徴点位置、FramePoseを推定すること。
 */
class BackendBase {
 public:
  explicit BackendBase(
      const std::shared_ptr<data::ThreadsafeMapDatabase>& map_database);
  virtual ~BackendBase() = default;

 protected:
  std::shared_ptr<data::ThreadsafeMapDatabase> map_database_;
};

}  // namespace vslam::backend