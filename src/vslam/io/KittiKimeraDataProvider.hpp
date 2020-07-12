//
// Created by ery on 2020/06/14.
//

#pragma once

#include <optional>

#include "DataProviderBase.hpp"
#include "KimeraFrontend.hpp"
#include "csv.h"

namespace vslam::dataprovider {

class KittiKimeraDataProvider : public KimeraDataProviderBase {
 private:
  using LogPack = std::tuple<uint64_t,    // save time
                             std::string  // path to image
                             >;

 public:
  class Parameter {
   public:
    Parameter();

    /**
     * @brief Path to dataset
     */
    std::string kitti_dataset_root_;
    std::string mask_image_;
  };

  /**
   * @param path_to_dataset_root : Specify the path to the root of Kitti. ex)
   * path/to/dataset/data_odometry_gray_dataset_sequence/00
   */
  KittiKimeraDataProvider(const std::string& path_to_dataset_root,
                          const std::string& path_to_mask_image = "");
  KittiKimeraDataProvider(const Parameter& parameter);

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

  Pose_t GetSensorPose();

 private:
  std::string GetPathToImageByIndexAndZeroFillNum(
      const std::string& path_to_image_prefix,
      const std::string& path_to_image_postfix,
      int32_t zero_fill_qty,
      int32_t image_index,
      const std::string& image_type);
  std::string Basename(const std::string& path);
  std::vector<LogPack> LogParser(const std::string& path_to_csv);
  std::unique_ptr<data::CameraModelBase> SetCameraParameters();

  std::string path_to_dataset_root_;
  std::string path_to_calibration_file_;

  cv::Mat mask_image_;

  uint64_t last_index_;
  std::unique_ptr<data::CameraModelBase> camera_model_;
  std::vector<LogPack> log_stored_;

  vslam::Pose_t pose_body_T_sensor_;
};

}  // namespace vslam::dataprovider