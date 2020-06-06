//
// Created by ery on 2020/04/24.
//

#include "BackendBase.hpp"
vslam::backend::BackendBase::BackendBase(
    const std::shared_ptr<data::ThreadsafeMapDatabase>& map_database)
    : map_database_(map_database) {
  ;
}
