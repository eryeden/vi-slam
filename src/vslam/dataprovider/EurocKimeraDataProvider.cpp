//
// Created by ery on 2020/05/03.
//

#include "EurocKimeraDataProvider.hpp"

#include <spdlog/spdlog.h>
#include <yaml-cpp/yaml.h>

#include "DataProviderBase.hpp"
#include "nlohmann/json.hpp"

using namespace vslam;
using namespace vslam::data;
using namespace vslam::dataprovider;

// EurocKimeraDataProvider::EurocKimeraDataProvider(
//    const std::string& path_to_dataset_root,
//    const std::string& path_to_calibration_file) {}

EurocKimeraDataProvider::EurocKimeraDataProvider(
    const std::string& path_to_dataset_root,
    const std::string& path_to_calibration_file)
    : path_to_dataset_root_(path_to_dataset_root),
      path_to_calibration_file_(path_to_calibration_file),
      last_index_(0) {
  // Parse CSV
  log_stored_ = LogParser(path_to_dataset_root + "/mav0/cam0/data.csv");
  // Load camera parameters
  camera_model_ = ParseCameraParameters(path_to_calibration_file_);
}

std::optional<frontend::KimeraFrontendInput>
EurocKimeraDataProvider::GetInput() {
  if (last_index_ >= log_stored_.size()) {
    spdlog::info("{}:{} : Reached the last line.", __FILE__, __FUNCTION__);
    return std::nullopt;
  }

  std::string path_to_image = path_to_dataset_root_ + "/mav0/cam0/data/" +
                              std::get<1>(log_stored_[last_index_]);

  frontend::KimeraFrontendInput frontend_input(
      static_cast<double>(std::get<0>(log_stored_[last_index_])) * 1e-9,
      cv::imread(path_to_image),
      camera_model_);

  last_index_++;
  return frontend_input;
}

std::optional<frontend::KimeraFrontendInput> EurocKimeraDataProvider::GetInput(
    uint64_t index) {
  if (index >= log_stored_.size()) {
    spdlog::info("{}:{} : Reached the last line.", __FILE__, __FUNCTION__);
    return std::nullopt;
  }

  std::string path_to_image = path_to_dataset_root_ + "/mav0/cam0/data/" +
                              std::get<1>(log_stored_[index]);

  frontend::KimeraFrontendInput frontend_input(
      static_cast<double>(std::get<0>(log_stored_[index])) * 1e-9,
      cv::imread(path_to_image),
      camera_model_);

  return frontend_input;
}

std::string EurocKimeraDataProvider::GetPathToImageByIndexAndZeroFillNum(
    const std::string& path_to_image_prefix,
    const std::string& path_to_image_postfix,
    int32_t zero_fill_qty,
    int32_t image_index,
    const std::string& image_type) {
  std::ostringstream sout;
  sout << std::setfill('0') << std::setw(zero_fill_qty) << image_index;
  return path_to_image_prefix + sout.str() + path_to_image_postfix + image_type;
}

std::string EurocKimeraDataProvider::Basename(const std::string& path) {
  return path.substr(path.find_last_of('/') + 1);
}

std::vector<EurocKimeraDataProvider::LogPack>
EurocKimeraDataProvider::LogParser(const std::string& path_to_csv) {
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

std::unique_ptr<data::CameraModelBase>
EurocKimeraDataProvider::ParseCameraParameters(
    const std::string& path_to_calibration_file) {
  // Read calib file
  std::fstream calib_file_input(path_to_calibration_file);
  nlohmann::json calib_json;
  calib_file_input >> calib_json;

  int32_t resolution_width, resolution_height;
  resolution_width = calib_json["value0"]["resolution"][0][0];
  resolution_height = calib_json["value0"]["resolution"][0][1];

  std::string camera_model_name =
      calib_json["value0"]["intrinsics"][0]["camera_type"];
  if (camera_model_name == "ds") {
    double fx = calib_json["value0"]["intrinsics"][0]["intrinsics"]["fx"];
    double fy = calib_json["value0"]["intrinsics"][0]["intrinsics"]["fy"];
    double cx = calib_json["value0"]["intrinsics"][0]["intrinsics"]["cx"];
    double cy = calib_json["value0"]["intrinsics"][0]["intrinsics"]["cy"];
    double xi = calib_json["value0"]["intrinsics"][0]["intrinsics"]["xi"];
    double alpha = calib_json["value0"]["intrinsics"][0]["intrinsics"]["alpha"];
    return std::make_unique<data::DoubleSphereCameraModel>(
        0, resolution_width, resolution_height, 0, fx, fy, cx, cy, xi, alpha);
  } else {
    spdlog::warn("{}:{} [{}] Specified camera model not implemented yet.",
                 __FILE__,
                 __FUNCTION__,
                 camera_model_name);
    std::exception();
    return std::unique_ptr<data::CameraModelBase>();
  }
}
