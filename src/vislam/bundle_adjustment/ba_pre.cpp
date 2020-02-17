//
// Created by ery on 2020/02/17.
//

#include "ba_pre.hpp"

using namespace vislam::ba;

/**
 * @brief 初期FrameIDにはMAX値を代入しておく
 */
ba_observation::ba_observation()
:frame_id(std::numeric_limits<uint64_t>::max())
{
;
}


