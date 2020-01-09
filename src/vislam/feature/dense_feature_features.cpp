#include "dense_feature_extructor.hpp"

using namespace dense_feature;

feature_in_frame::feature_in_frame(
    const uint64_t frame_id,
    const std::vector<uint64_t> &input_feature_ids,
    const std::vector<Eigen::Vector2i, Eigen::aligned_allocator<Eigen::Vector2i>> &input_features)
    : frameID(frame_id),
      features(input_features),
      featureIDs(input_feature_ids)
{
}

feature_in_frame::feature_in_frame()
    : feature_in_frame(0, {}, {})
{
}
