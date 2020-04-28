//
// Created by ery on 2020/04/25.
//

#pragma once

#include <mutex>

#include "Frame.hpp"
#include "Landmark.hpp"
#include "type_defines.hpp"

namespace vslam::data {

/**
 * @brief Mapが保持する、FrameとLandmarkを登録する。
 * @details
 * 基本的にすべてのFrame、Landmarkがshared_ptrによって保持される。
 *
 * ただし、Shared_prtをアクセス要求を送りつけてくる各所にばらまくと、どこかでShared_ptrを保持しつつけ、
 * Map DB側からの開放ができなくなる可能性がある。
 * Shared_ptrの場合、確保したメモリを開放したいとしても、shared_ptrの参照回数をデクリメントすることしかできないため、
 * どこかのポインタ所有者が生き残っている場合に開放できない懸念点がある。
 *
 * このため、MapDBが全てのメモリ所有権を主張できるように、
 * 入力は、unique_ptr
 * 内部保持は、shared_ptr
 * 出力は、weak_ptr
 * として実装する。
 * こうすることで、メモリ所有権はMapDBにしかないことになるので、MapDBがメモリリークを引き起こす可能性はなくなるはず。
 *
 */
class ThreadsafeMapDatabase {
 public:
  ThreadsafeMapDatabase();

  void AddFrame(FrameUniquePtr& frame_ptr);
  void EraseFrame(database_index_t frame_id);
  FrameDatabaseWeak GetAllFrames() const;
  /**
   * @brief
   * Frameへの参照をweak_ptrで返す。IDが存在しない場合、空のweak_ptrを返す。
   * @param frame_id
   * @return
   */
  FrameWeakPtr GetFrame(database_index_t frame_id) const;

  void AddLandmark(LandmarkUniquePtr& landmark_ptr);
  void EraseLandmark(database_index_t landmark_id);
  LandmarkDatabaseWeak GetAllLandmarks() const;
  LandmarkWeakPtr GetLandmark(database_index_t landmark_id) const;

  void Clear();

  std::atomic<database_index_t> latest_frame_id_;
  std::atomic<database_index_t> latest_key_frame_id_;

 private:
  // Frame database; key : Frame id, value : Pointer to the related frame.
  FrameDatabaseShared frame_database_;
  // Landmark database; key : Landmark id, value : Pointer to the related
  // Landmark.
  LandmarkDatabaseShared landmark_database_;

  // Map DB全体をLockするmutex
  mutable std::mutex mutex_map_access_;
};

};  // namespace vslam::data
