#include "feature_in_frame.hpp"

using namespace dense_feature;

feature_in_frame::feature_in_frame(
    const uint64_t frame_id,
    const std::vector<uint64_t> &input_feature_ids,
    const std::vector<Eigen::Vector2i> &input_features)
    : frameID(frame_id),
      features(input_features),
      featureIDs(input_feature_ids),
      featureMasks(0),
      featuresIn3d(0)
{
}

feature_in_frame::feature_in_frame()
    : feature_in_frame(0, {}, {})
{
}
