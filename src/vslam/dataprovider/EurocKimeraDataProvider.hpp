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
   * @param path_to_calibration_file : Specify the path to a basalt calibration
   * file.
   */
  EurocKimeraDataProvider(const std::string& path_to_dataset_root,
                          const std::string& path_to_calibration_file,
                          const std::string& path_to_mask_image = "");

  /**
   * @brief Get data and increment a internal line index.
   * @return KimeraFrontendInputRadialTangentialCameraModel
   */
  std::optional<frontend::KimeraFrontendInput> GetInput() override;
  /**
   * @brief Get data by the index specified.
   * @param index : A line number of data frame which you want.
   * @return KimeraFrontendInputRadialTangentialCameraModel
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
  std::unique_ptr<data::CameraModelBase> ParseCameraParameters(
      const std::string& path_to_calibration_file);

  std::string path_to_dataset_root_;
  std::string path_to_calibration_file_;

  cv::Mat mask_image_;

  uint64_t last_index_;
  std::unique_ptr<data::CameraModelBase> camera_model_;
  std::vector<LogPack> log_stored_;
};

}  // namespace vslam::dataprovider