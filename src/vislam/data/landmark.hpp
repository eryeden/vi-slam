#pragma once

#include "type_defines.hpp"

namespace vislam::data
{

class landmark
{

public:


        landmark(uint64_t id,
             const std::unordered_set<uint64_t> & observed_frame_id,
             const Vec3_t &position_in_world, bool is_outlier, bool is_tracking);
        explicit landmark(uint64_t id);
        landmark();


        uint64_t id;
        std::unordered_set<uint64_t> observedFrameId;
        Vec3_t positionInWorld;
        bool isOutlier;
        bool isTracking;




private:


};

} // namespace vislam