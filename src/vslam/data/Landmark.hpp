#pragma once

#include <atomic>
#include <memory>
#include <mutex>

#include "type_defines.hpp"

namespace vslam::data {

class Landmark {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Landmark(database_index_t id,
           const std::set<database_index_t>& observed_frame_id,
           const Vec3_t& position_in_world,
           bool is_outlier,
           bool is_initialized);

  Landmark();

  Landmark(const Landmark& landmark);

  //! Landmarkの被観測関係の取得、設定
  void SetObservedFrameIndex(database_index_t frame_index);
  std::set<database_index_t> GetAllObservedFrameIndex() const;

  //! Landmark推定位置関係
  void SetLandmarkPosition(const Vec3_t& position_in_world);
  Vec3_t GetLandmarkPosition() const;

  //! Landmark ID, これは変更されないはずなので、Multi
  //! threadアクセスを考えてconstにする
  const database_index_t landmark_id_;

  //! atomicで用意する
  std::atomic_bool is_initialized_;
  std::atomic_bool is_outlier_;

 private:
  //! 観測関係のデータ
  mutable std::mutex mutex_observation_;
  std::set<database_index_t> observed_frame_id_;

  //! Landmark位置関係
  mutable std::mutex mutex_position_;
  Vec3_t position_in_world_;
};

using LandmarkSharedPtr = std::shared_ptr<Landmark>;
using LandmarkUniquePtr = std::unique_ptr<Landmark>;
using LandmarkWeakPtr = std::weak_ptr<Landmark>;
using LandmarkDatabaseWeak =
    std::unordered_map<database_index_t, LandmarkWeakPtr>;
using LandmarkDatabaseShared =
    std::unordered_map<database_index_t, LandmarkSharedPtr>;

}  // namespace vslam::data