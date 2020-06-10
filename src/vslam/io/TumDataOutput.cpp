//
// Created by ery on 2020/06/10.
//

#include "TumDataOutput.hpp"

#include "fmt/format.h"

using namespace vslam;

vslam::dataoutput::TumDataOutput::TumDataOutput(const std::string& path_to_save)
    : path_to_save_(path_to_save) {
  output_stream_.open(path_to_save_, std::ios::out);
}

void vslam::dataoutput::TumDataOutput::SavePose(double timestamp_sec,
                                                vslam::Pose_t pose) {
  Quat_t q(pose.rotationMatrix());
  auto translation = pose.translation();

  auto output_string = fmt::format("{} {} {} {} {} {} {} {}\n",
                                   timestamp_sec,
                                   translation[0],
                                   translation[1],
                                   translation[2],
                                   q.x(),
                                   q.y(),
                                   q.z(),
                                   q.w());
  output_stream_ << output_string;
  output_stream_.flush();
}
