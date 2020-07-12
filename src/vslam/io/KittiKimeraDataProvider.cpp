//
// Created by ery on 2020/06/14.
//

#include "KittiKimeraDataProvider.hpp"

#include <fmt/format.h>

using namespace vslam;
using namespace vslam::dataprovider;
using namespace vslam::frontend;

KittiKimeraDataProvider::Parameter::Parameter() {
  kitti_dataset_root_ = "/home/ery/subspace/docker_work/dataset/data_odometry_gray/dataset/sequences/03";
  mask_image_ = "";
}

vslam::dataprovider::KittiKimeraDataProvider::KittiKimeraDataProvider(
    const std::string& path_to_dataset_root,
    const std::string& path_to_mask_image)
    : path_to_dataset_root_(path_to_dataset_root), last_index_(0) {
  path_to_calibration_file_ = path_to_dataset_root_ + "/calib.txt";

  // Parse dataset, generate the tuple<time, path_to_image>
  log_stored_ = LogParser(path_to_dataset_root + "/times.txt");
  // Parse sensor pose
  pose_body_T_sensor_ = Pose_t(Mat44_t::Identity());
  // Load camera parameters
  camera_model_ = SetCameraParameters();

  // Load mask image
  if (path_to_mask_image != "") {
    cv::Mat mask_image_raw =
        cv::imread(path_to_mask_image, cv::IMREAD_GRAYSCALE);
    cv::threshold(mask_image_raw, mask_image_, 10, 255, CV_THRESH_BINARY);
  }
}

vslam::dataprovider::KittiKimeraDataProvider::KittiKimeraDataProvider(const Parameter& parameter)
: KittiKimeraDataProvider(parameter.kitti_dataset_root_, parameter.mask_image_)
{
}

std::optional<frontend::KimeraFrontendInput>
vslam::dataprovider::KittiKimeraDataProvider::GetInput() {
  if (last_index_ >= log_stored_.size()) {
    spdlog::info("{}:{} : Reached the last line.", __FILE__, __FUNCTION__);
    return std::nullopt;
  }

  std::string path_to_image = path_to_dataset_root_ + "/image_0/" +
                              std::get<1>(log_stored_[last_index_]);

  cv::Mat load_raw = cv::imread(path_to_image, cv::IMREAD_GRAYSCALE);
  //  cv::Mat store_img(load_raw.size(), CV_8U);
  //  load_raw.convertTo(store_img, CV_8U);

  frontend::KimeraFrontendInput frontend_input(
      static_cast<double>(std::get<0>(log_stored_[last_index_])) * 1e-9,
      load_raw,  // & mask_image_,
      mask_image_,
      camera_model_);

  last_index_++;
  return frontend_input;
}
std::optional<frontend::KimeraFrontendInput>
vslam::dataprovider::KittiKimeraDataProvider::GetInput(uint64_t index) {
  if (index >= log_stored_.size()) {
    spdlog::info("{}:{} : Reached the last line.", __FILE__, __FUNCTION__);
    return std::nullopt;
  }

  std::string path_to_image =
      path_to_dataset_root_ + "/image_0/" + std::get<1>(log_stored_[index]);

  cv::Mat load_raw = cv::imread(path_to_image, cv::IMREAD_GRAYSCALE);

  frontend::KimeraFrontendInput frontend_input(
      static_cast<double>(std::get<0>(log_stored_[index])) * 1e-9,
      load_raw,  // & mask_image_,
      mask_image_,
      camera_model_);

  return frontend_input;
}
vslam::Pose_t vslam::dataprovider::KittiKimeraDataProvider::GetSensorPose() {
  return pose_body_T_sensor_;
}

std::string vslam::dataprovider::KittiKimeraDataProvider::
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
std::string vslam::dataprovider::KittiKimeraDataProvider::Basename(
    const std::string& path) {
  return path.substr(path.find_last_of('/') + 1);
}
std::vector<KittiKimeraDataProvider::LogPack>
vslam::dataprovider::KittiKimeraDataProvider::LogParser(
    const std::string& path_to_csv) {
  std::vector<LogPack> logs;
  io::CSVReader<1> in_csv(path_to_csv);
  LogPack tmp_log_pack;
  double timestamp;
  int64_t counter = 0;
  while (in_csv.read_row(timestamp)) {
    std::get<0>(tmp_log_pack) = static_cast<uint64_t>(timestamp * 1e9);
    std::get<1>(tmp_log_pack) = fmt::format("{:06}.png", counter++);
    std::cout << std::get<1>(tmp_log_pack) << std::endl;
    logs.emplace_back(tmp_log_pack);
  }

  return logs;
}
std::unique_ptr<data::CameraModelBase>
vslam::dataprovider::KittiKimeraDataProvider::SetCameraParameters() {
  // 7.188560000000e+02 0.000000000000e+00 6.071928000000e+02 0.000000000000e+00
  // 0.000000000000e+00 7.188560000000e+02 1.852157000000e+02 0.000000000000e+00
  // 0.000000000000e+00 0.000000000000e+00 1.000000000000e+00 0.000000000000e+00

  return std::make_unique<data::PinholeCameraModel>(0,
                                                    1241,
                                                    376,
                                                    0,
                                                    7.188560000000e+02,
                                                    7.188560000000e+02,
                                                    6.071928000000e+02,
                                                    1.852157000000e+02);
}
