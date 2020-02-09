#pragma once

#include "type_defines.hpp"

namespace vislam::data
{


struct frame
{
public:
    /**
     * @brief 観測したフレームの情報を保存する
     * @details
     *
     *
     * @param id
     * @param observed_frame_id
     * @param position_in_world
     * @param is_outlier
     * @param is_tracking
     */
    frame(uint64_t id,
          const std::unordered_set<uint64_t> & observed_frame_id,
          const Vec3_t &position_in_world, bool is_outlier, bool is_tracking);
    explicit frame(uint64_t id);
    frame();

    uint64_t id;
    std::unordered_set<uint64_t> observedFrameId;
    Vec3_t positionInWorld;
    bool isOutlier;
    bool isTracking;

private:
};

} // namespace vislam
