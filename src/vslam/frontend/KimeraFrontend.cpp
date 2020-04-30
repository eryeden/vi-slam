//
// Created by ery on 2020/04/30.
//

#include "KimeraFrontend.hpp"

using namespace vslam::data;
using namespace vslam::frontend;

/**
 * @brief Kimera-VIOベースの単眼Frontend
 * @param threadsafe_map_database
 */
KimeraFrontend::KimeraFrontend(
    const std::shared_ptr<data::ThreadsafeMapDatabase>& threadsafe_map_database)
    : FrontendBase(threadsafe_map_database) {}

/**
 * @brief Implementation of Kimera-based Vision-Frontend
 * @details
 * Frontendのメインループとなるので、ここを起点にFrontendの全処理が記述される
 * @param image
 * @return
 */
FrontendStatus KimeraFrontend::FeedImage(const cv::Mat& image) {
  return FrontendStatus();
}
