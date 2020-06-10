//
// Created by ery on 2020/05/03.
//

#pragma once

#include <optional>

#include "KimeraFrontend.hpp"
#include "csv.h"

namespace vslam::dataprovider {
class KimeraDataProviderBase {
 public:
  KimeraDataProviderBase(){};

  virtual std::optional<frontend::KimeraFrontendInput> GetInput() = 0;
  virtual std::optional<frontend::KimeraFrontendInput> GetInput(
      uint64_t index) = 0;

 private:
};
}  // namespace vslam::dataprovider