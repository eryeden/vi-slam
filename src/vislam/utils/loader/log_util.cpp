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
void LogPlayer_vio_dataset::generate_database_form_file(LogPlayer_vio_dataset::frame_database_t &output_frame_database,
                                                        LogPlayer_vio_dataset::landmark_database_t &output_landmark_database) {

}
LogPlayer_vio_dataset::frame_database_t LogPlayer_vio_dataset::make_frame_database(const std::string &path_to_frame_positon) {
  return LogPlayer_vio_dataset::frame_database_t();  using frame_database_t = std::unordered_map<uint64_t, vislam::data::frame>;
  using landmark_database_t = std::unordered_map<uint64_t, vislam::data::landmark>;

}
LogPlayer_vio_dataset::landmark_database_t LogPlayer_vio_dataset::make_landmakr_database(const std::string &path_to_frame_positon) {
  return LogPlayer_vio_dataset::landmark_database_t();
}
void LogPlayer_vio_dataset::add_landmark_observaton(const std::string &pathToKeyframeFile,
                                                    uint64_t frame_id,
                                                    LogPlayer_vio_dataset::frame_database_t &frame_database,
                                                    LogPlayer_vio_dataset::landmark_database_t &landmark_database) {

}


