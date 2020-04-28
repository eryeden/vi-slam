#pragma once

#include <atomic>
#include <memory>
#include <mutex>

#include "Camera.hpp"
#include "Landmark.hpp"
#include "type_defines.hpp"

namespace vslam::data {


class Frame {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Frame(database_index_t id,
        double timestamp,
        bool is_keyframe,
        const PinholeCameraModel& camera_parameters,
        const std::set<database_index_t>& observing_feature_id,
        const EigenAllocatedUnorderedMap<database_index_t, Vec2_t>&
            observing_feature_points_in_device);

  void SetCameraPose(const Vec3_t& position, const Quat_t& orientation);
  Vec3_t GetCameraPosition() const;
  Quat_t GetCameraOrientation() const;
  void GetCameraPose(Vec3_t& position, Quat_t& orientation) const;

  //! 観測したLandmarkを登録する。初見のIDなら登録、もし登録済みのIDならReplaceになる。
  void SetLandmark(const LandmarkSharedPtr& landmark);
  void EraseLandmark(database_index_t landmark_index);
  LandmarkDatabaseWeak GetAllLandmarks() const;
  LandmarkWeakPtr GetLandmark(database_index_t landmark_id) const;

  //! FrameごとのID
  //! 一回定義されたら変更されないはずなので定数定義
  //! constであればwriteアクセスは発生しないので、Multi-thread実装においてガードは必要ない
  const database_index_t frame_id_;

  //! Frameが観測された時点でのTimestamp
  const double timestamp_;

  //! @note あとから変更されうるのでstd::atomicによるガードを行う
  //! FrameはKeyFrameか？TrueでKeyFrameになる
  std::atomic_bool is_keyframe_;

  //! Pinholeカメラパラメータ
  const PinholeCameraModel camera_parameter_;

  //! @note
  //! 観測情報はFrame生成時から変わり得ないのでconstで保持する。故にMutexなどのガードは必要ない
  //! 観測したLandmark IDを保持する
  const std::set<database_index_t> observing_feature_id_;
  //! 観測した画像上のFeature positionを保持する
  const EigenAllocatedUnorderedMap<database_index_t, Vec2_t>
      observing_feature_point_in_device_;

 private:
  //! @note
  //! カメラPoseは後に修正されうるのでconst定義ができない。ゆえにアクセス時Mutexなどを利用したガードが必要になる。
  //! カメラ関係用のMutex
  //! @note mutableある理由 : https://yohhoy.hatenadiary.jp/entry/20130803/p1
  //! @note mutableになっているとconstメンバ関数からでも変更が可能になる。
  //! @note mutex lockされる変数は、Read/WriteともにmutexのLockが必要になる。
  //! @note ここで、Readの場合はconstの修飾をして
  mutable std::mutex mutex_camera_pose_;
  //! カメラ位置
  Vec3_t camera_position_;
  //! カメラ姿勢
  Quat_t camera_orientation_;

  //! Landmark関係のMutex
  mutable std::mutex mutex_landmark_;
  //! Landmark
  LandmarkDatabaseWeak landmarks_;
};

using FrameSharedPtr = std::shared_ptr<Frame>;
using FrameUniquePtr = std::unique_ptr<Frame>;
using FrameWeakPtr = std::weak_ptr<Frame>;
using FrameDatabaseWeak = std::unordered_map<database_index_t, FrameWeakPtr>;
using FrameDatabaseShared =
    std::unordered_map<database_index_t, FrameSharedPtr>;

}  // namespace vslam::data
