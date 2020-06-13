//
// Created by ery on 2020/06/12.
//

#include "LogDataOutput.hpp"

#include <spdlog/spdlog.h>

#include <filesystem>

vslam::dataoutput::LogDataOutput::LogDataOutput(
    const std::string& path_to_save_dir)
    : path_to_save_dir_(path_to_save_dir) {
  path_to_frame_dump_dir_from_log_root_ = "frames";
  bool result = std::filesystem::create_directory(
      path_to_save_dir_ + "/" + path_to_frame_dump_dir_from_log_root_);
  if (!result) {
    spdlog::error(
        "{} : Failed to create directory[{}].",
        __FUNCTION__,
        path_to_save_dir_ + "/" + path_to_frame_dump_dir_from_log_root_);
  }
}