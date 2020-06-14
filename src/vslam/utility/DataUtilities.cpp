//
// Created by ery on 2020/06/11.
//

#include "DataUtilities.hpp"

using namespace vslam;
using namespace vslam::utility;
using namespace vslam::data;

data::InternalMaterials vslam::utility::GenerateInternalsFromFrame(
    const Frame& frame,
    const std::shared_ptr<data::ThreadsafeMapDatabase> map_database) {
  data::InternalMaterials materials = frame.internal_materials_;

  materials.frame_id_ = frame.frame_id_;
  materials.is_keyframe_ = frame.is_keyframe_;
  materials.timestamp_ = frame.timestamp_;
  materials.observing_feature_id_ = frame.observing_feature_id_;
  materials.observing_feature_point_in_device_ =
      frame.observing_feature_point_in_device_;
  materials.observing_feature_bearing_in_camera_frame_ =
      frame.observing_feature_bearing_in_camera_frame_;
  materials.feature_point_age_ = frame.feature_point_age_;
  materials.camera_pose_ = frame.GetCameraPose();

  for (const auto lm_id : frame.observing_feature_id_) {
    auto lm_ptr = map_database->GetLandmark(lm_id).lock();
    if (lm_ptr) {
      materials.landmarks_.insert(std::make_pair(lm_id, *lm_ptr));
    }
  }

  return materials;
}
