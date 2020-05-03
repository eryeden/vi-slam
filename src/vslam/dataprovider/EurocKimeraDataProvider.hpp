//
// Created by ery on 2020/05/03.
//

#pragma once

#include <optional>

#include "DataProviderBase.hpp"
#include "KimeraFrontend.hpp"
#include "csv.h"

namespace vslam::dataprovider {

class EurocKimeraDataProvider : public KimeraDataProviderBase {
 private:
  using LogPack = std::tuple<uint64_t,    // save time
                             std::string  // path to image
                             >;

 public:
  /**
   * @param path_to_dataset_root : Specify the path to the root of EUROC. ex)
   * path/to/dataset/V1_01_easy
   */
  explicit EurocKimeraDataProvider(const std::string& path_to_dataset_root);

  /**
   * @brief Get data and increment a internal line index.
   * @return KimeraFrontendInput
   */
  std::optional<frontend::KimeraFrontendInput> GetInput() override;
  /**
   * @brief Get data by the index specified.
   * @param index : A line number of data frame which you want.
   * @return KimeraFrontendInput
   */
  std::optional<frontend::KimeraFrontendInput> GetInput(
      uint64_t index) override;

 private:
  std::string GetPathToImageByIndexAndZeroFillNum(
      const std::string& path_to_image_prefix,
      const std::string& path_to_image_postfix,
      int32_t zero_fill_qty,
      int32_t image_index,
      const std::string& image_type);
  std::string Basename(const std::string& path);
  std::vector<LogPack> LogParser(const std::string& path_to_csv);
  data::PinholeCameraModel ParseCameraParameters(
      const std::string& path_to_camera_parameters);

  std::string path_to_dataset_root_;
  uint64_t last_index_;
  data::PinholeCameraModel camera_model_;
  std::vector<LogPack> log_stored_;
};
}  // namespace vslam::dataprovider