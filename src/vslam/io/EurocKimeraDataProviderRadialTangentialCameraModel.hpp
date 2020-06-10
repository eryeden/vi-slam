//
// Created by ery on 2020/05/03.
//

#pragma once

#include <optional>

#include "DataProviderBase.hpp"
#include "KimeraFrontend.hpp"
#include "csv.h"

namespace vslam::dataprovider {

class EurocKimeraDataProviderRadialTangentialCameraModel {
 private:
  using LogPack = std::tuple<uint64_t,    // save time
                             std::string  // path to image
                             >;

 public:
  /**
   * @param path_to_dataset_root : Specify the path to the root of EUROC. ex)
   * path/to/dataset/V1_01_easy
   */
  explicit EurocKimeraDataProviderRadialTangentialCameraModel(
      const std::string& path_to_dataset_root);

  /**
   * @brief Get data and increment a internal line index.
   * @return KimeraFrontendInputRadialTangentialCameraModel
   */
  std::optional<frontend::KimeraFrontendInputRadialTangentialCameraModel>
  GetInput();
  /**
   * @brief Get data by the index specified.
   * @param index : A line number of data frame which you want.
   * @return KimeraFrontendInputRadialTangentialCameraModel
   */
  std::optional<frontend::KimeraFrontendInputRadialTangentialCameraModel>
  GetInput(uint64_t index);

 private:
  std::string GetPathToImageByIndexAndZeroFillNum(
      const std::string& path_to_image_prefix,
      const std::string& path_to_image_postfix,
      int32_t zero_fill_qty,
      int32_t image_index,
      const std::string& image_type);
  std::string Basename(const std::string& path);
  std::vector<LogPack> LogParser(const std::string& path_to_csv);
  data::RadialTangentialCameraModel ParseCameraParameters(
      const std::string& path_to_camera_parameters);

  std::string path_to_dataset_root_;
  uint64_t last_index_;
  data::RadialTangentialCameraModel camera_model_;
  std::vector<LogPack> log_stored_;
};

}  // namespace vslam::dataprovider