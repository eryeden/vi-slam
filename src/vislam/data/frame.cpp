#include "frame.hpp"

using namespace vislam::data;


frame::frame(uint64_t id_, const std::unordered_set<uint64_t> &observing_feature_id,
                   const vislam::eigen_allocated_unordered_map<uint64_t, vislam::Vec2_t> &observing_feature_point_in_device,
                   const vislam::Vec3_t &camera_position, const vislam::Quat_t &camera_attitude,
                   const vislam::Mat33_t &camera_intrinsic_parameter) {
    id = id_;
    observingFeatureId = observing_feature_id;
    observingFeaturePointInDevice = observing_feature_point_in_device;
    cameraPosition = camera_position;
    cameraAttitude = camera_attitude;
    cameraIntrinsicParameter = camera_intrinsic_parameter;
}

frame::frame(uint64_t id)
        :frame(id,
                  {},
                  {},
                  {0,0,0},
                  {0,0,0,1},
                  Eigen::MatrixXd::Identity(3,3))
{
    ;
}

frame::frame()
        :frame(std::numeric_limits<uint64_t>::max())
{
    ;
}
