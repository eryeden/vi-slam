#include "landmark.hpp"


using namespace vislam::data;


landmark::landmark(uint64_t id_,
        const std::unordered_set<uint64_t> & observed_frame_id,
        const Vec3_t &position_in_world,
        bool is_outlier,
        bool is_tracking){
    id = id_;

    observedFrameId = observed_frame_id;
    positionInWorld = position_in_world;
    isOutlier = is_outlier;
    isTracking = is_tracking;

}
landmark::landmark(uint64_t id_)
: landmark(id_, {}, {0,0,0}, false, false)
{
;
}
landmark::landmark()
: landmark(std::numeric_limits<uint64_t>::max())
{
;
}



