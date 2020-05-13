//
// Created by ery on 2020/05/03.
//

#include "EurocKimeraDataProviderRadialTangentialCameraModel.hpp"

#include <spdlog/spdlog.h>
#include <yaml-cpp/yaml.h>

#include "DataProviderBase.hpp"

using namespace vslam;
using namespace vslam::data;
using namespace vslam::dataprovider;

EurocKimeraDataProviderRadialTangentialCameraModel::
    EurocKimeraDataProviderRadialTangentialCameraModel(
        const std::string& path_to_dataset_root)
    : path_to_dataset_root_(path_to_dataset_root), last_index_(0) {
  // Parse CSV
  log_stored_ = LogParser(path_to_dataset_root + "/mav0/cam0/data.csv");
  // Load camera parameters
  camera_model_ =
      ParseCameraParameters(path_to_dataset_root + "/mav0/cam0/sensor.yaml");
}

std::optional<frontend::KimeraFrontendInputRadialTangentialCameraModel>
EurocKimeraDataProviderRadialTangentialCameraModel::GetInput() {
  if (last_index_ >= log_stored_.size()) {
    spdlog::info("{}:{} : Reached the last line.", __FILE__, __FUNCTION__);
    return std::nullopt;
  }

  frontend::KimeraFrontendInputRadialTangentialCameraModel frontend_input;

  frontend_input.timestamp_ =
      static_cast<double>(std::get<0>(log_stored_[last_index_])) * 1e-9;
  frontend_input.camera_model_ = camera_model_;

  std::string path_to_image = path_to_dataset_root_ + "/mav0/cam0/data/" +
                              std::get<1>(log_stored_[last_index_]);
  frontend_input.frame_ = cv::imread(path_to_image);

  last_index_++;
  return frontend_input;
}

std::optional<frontend::KimeraFrontendInputRadialTangentialCameraModel>
EurocKimeraDataProviderRadialTangentialCameraModel::GetInput(uint64_t index) {
  if (index >= log_stored_.size()) {
    spdlog::info("{}:{} : Reached the last line.", __FILE__, __FUNCTION__);
    return std::nullopt;
  }

  frontend::KimeraFrontendInputRadialTangentialCameraModel frontend_input;

  frontend_input.timestamp_ =
      static_cast<double>(std::get<0>(log_stored_[index])) * 1e-9;
  frontend_input.camera_model_ = camera_model_;

  std::string path_to_image = path_to_dataset_root_ + "/mav0/cam0/data/" +
                              std::get<1>(log_stored_[index]);
  frontend_input.frame_ = cv::imread(path_to_image);

  return frontend_input;
}

std::string EurocKimeraDataProviderRadialTangentialCameraModel::
    GetPathToImageByIndexAndZeroFillNum(
        const std::string& path_to_image_prefix,
        const std::string& path_to_image_postfix,
        int32_t zero_fill_qty,
        int32_t image_index,
        const std::string& image_type) {
  std::ostringstream sout;
  sout << std::setfill('0') << std::setw(zero_fill_qty) << image_index;
  return path_to_image_prefix + sout.str() + path_to_image_postfix + image_type;
}

std::string EurocKimeraDataProviderRadialTangentialCameraModel::Basename(
    const std::string& path) {
  return path.substr(path.find_last_of('/') + 1);
}

std::vector<EurocKimeraDataProviderRadialTangentialCameraModel::LogPack>
EurocKimeraDataProviderRadialTangentialCameraModel::LogParser(
    const std::string& path_to_csv) {
  std::vector<LogPack> logs;

  io::CSVReader<2> in_csv(path_to_csv);
  // in_csv.set_header("timestamp", "filename");
  // in_csv.read_header(io::ignore_column, "timestamp", "filename");
  in_csv.read_header(io::ignore_missing_column, "#timestamp [ns]", "filename");

  LogPack tmp_log_pack;
  while (
      in_csv.read_row(std::get<0>(tmp_log_pack), std::get<1>(tmp_log_pack))) {
    logs.emplace_back(tmp_log_pack);
  }
  return logs;
}

data::RadialTangentialCameraModel
EurocKimeraDataProviderRadialTangentialCameraModel::ParseCameraParameters(
    const std::string& path_to_camera_parameters) {
  YAML::Node config = YAML::LoadFile(path_to_camera_parameters);

  int32_t resolution_width, resolution_height;
  resolution_width = config["resolution"][0].as<int32_t>();
  resolution_height = config["resolution"][1].as<int32_t>();

  double fx, fy, cx, cy;
  fx = config["intrinsics"][0].as<double>();
  fy = config["intrinsics"][1].as<double>();
  cx = config["intrinsics"][2].as<double>();
  cy = config["intrinsics"][3].as<double>();

  /**
   * @brief Load distortion coefficients
   * @note
   * The parameter k3 seems neglected in EUROC, so specify it as 0 currently.
   */
  double k1, k2, p1, p2, k3;
  k1 = config["distortion_coefficients"][0].as<double>();
  k2 = config["distortion_coefficients"][1].as<double>();
  p1 = config["distortion_coefficients"][2].as<double>();
  p2 = config["distortion_coefficients"][3].as<double>();
  k3 = 0;

  int32_t rate_hz;
  rate_hz = config["rate_hz"].as<int32_t>();

  data::RadialTangentialCameraModel camera_model;
  camera_model.id = 0;
  camera_model.width = resolution_width;
  camera_model.height = resolution_height;
  camera_model.fx = fx;
  camera_model.fy = fy;
  camera_model.cx = cx;
  camera_model.cy = cy;
  camera_model.k1 = k1;
  camera_model.k2 = k2;
  camera_model.p1 = p1;
  camera_model.p2 = p2;
  camera_model.k3 = k3;
  camera_model.fps = rate_hz;

  return camera_model;
}
