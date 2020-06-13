//
// Created by ery on 2020/06/11.
//

#pragma once
#include "Frame.hpp"
#include "Internals.hpp"
#include "Landmark.hpp"
#include "ThreadsafeContainer.hpp"

namespace vslam::utility {

data::InternalMaterials GenerateInternalsFromFrame(
    const data::Frame& frame,
    const std::shared_ptr<data::ThreadsafeMapDatabase> map_database);

}