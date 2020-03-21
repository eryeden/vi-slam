//
// Created by ery on 2020/03/21.
//

#include "log_util.h"

LogPlayer_vio_dataset::LogPlayer_vio_dataset(const std::string &path_to_log_dir)
: pathToLogDir(path_to_log_dir),
  filenameLandmarkPositon("all_points.txt"),
  filenameFramePosition("cam_pose.txt"),
  prefixKeyFrame("keyframe/all_points_"),
  postfixKeyFrame(".txt")
{
  ;
}


