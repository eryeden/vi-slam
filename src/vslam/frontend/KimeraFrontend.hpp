//
// Created by ery on 2020/04/30.
//

#pragma once

#include "FrontendBase.hpp"

namespace vslam::frontend {

class KimeraFrontend : public FrontendBase {
 public:
  KimeraFrontend(const std::shared_ptr<data::ThreadsafeMapDatabase>&
                     threadsafe_map_database);

  FrontendStatus FeedImage(const cv::Mat& image) override;

 private:
};

}  // namespace vslam::frontend