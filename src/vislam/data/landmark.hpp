#pragma once

#include "type_defines.hpp"

namespace vislam::data
{

class landmark
{

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW


    landmark(uint64_t id,
//             const std::unordered_set<uint64_t> & observed_frame_id,
             const std::set<uint64_t> & observed_frame_id,
             const Vec3_t &position_in_world, bool is_outlier, bool is_tracking);
    explicit landmark(uint64_t id);
    landmark();


    uint64_t id;
//        std::unordered_set<uint64_t> observedFrameId;
    /**
     * @brief std::unordered_setはstd::set_intersectionがうまく行かないので、std::setにしている。
     */
    std::set<uint64_t> observedFrameId;
    Vec3_t positionInWorld;
    bool isOutlier;
    bool isTracking;




private:


};

} // namespace vislam