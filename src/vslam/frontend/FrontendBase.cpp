//
// Created by ery on 2020/04/24.
//

#include "FrontendBase.hpp"
vslam::frontend::FrontendBase::FrontendBase(
    const std::shared_ptr<data::ThreadsafeMapDatabase>& map_database)
    : map_database_(map_database) {
  ;
}
