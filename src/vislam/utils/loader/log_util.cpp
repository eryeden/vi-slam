//
// Created by ery on 2020/03/21.
//

#include "log_util.h"
#include <iostream>
#include <vector>

struct frame_info {
  //! Timestamp[s]
  double time;
  //! Frame orientation represented in quaternion
  double qw, qx, qy, qz;
  //! Frame position
  double tx, ty, tz;
  //! Measured rotation velocity
  double gx, gy, gz;
  //! Measured acceleration
  double ax, ay, az;
};

struct landmark_info {
  //! Landmark position in world frame
  double x, y, z, dummy;
};

struct frame_observation_info {
  //! landmark position in world frame
  double lx, ly, lz, dummy;
  //! Observed landmark position in device frame
  double fx, fy;
};

LogPlayer_vio_dataset::LogPlayer_vio_dataset(const std::string &path_to_log_dir)
    : pathToLogDir(path_to_log_dir),
      filenameLandmarkPosition("all_points.txt"),
      filenameFramePosition("cam_pose.txt"),
      prefixKeyFrame("keyframe/all_points_"),
      postfixKeyFrame(".txt") {
  ;
}
void LogPlayer_vio_dataset::generate_database_form_file(LogPlayer_vio_dataset::frame_database_t &output_frame_database,
                                                        LogPlayer_vio_dataset::landmark_database_t &output_landmark_database) {
  frame_database_t frame_database = make_frame_database(pathToLogDir + "/" + filenameFramePosition);
  landmark_database_t landmark_database = make_landmark_database(pathToLogDir + "/" + filenameLandmarkPosition);

  for (const auto&[frame_id, frame] : frame_database) {
    std::string
        path_to_keyframe_file = pathToLogDir + "/" + prefixKeyFrame + std::to_string(frame_id) + postfixKeyFrame;
    add_landmark_observation(path_to_keyframe_file, frame_id, frame_database, landmark_database);
  }

  output_frame_database = frame_database;
  output_landmark_database = landmark_database;
}

LogPlayer_vio_dataset::frame_database_t LogPlayer_vio_dataset::make_frame_database(const std::string &path_to_frame_position) {
  frame_database_t frame_database;
  io::CSVReader<14, io::trim_chars<>, io::no_quote_escape<' '>> in_csv(path_to_frame_position);
//  io::CSVReader<14, io::no_quote_escape<' '>> in_csv(path_to_frame_position);
  in_csv.set_header("time", "qw", "qx", "qy", "qz", "tx", "ty", "tz", "gx", "gy", "gz", "ax", "ay", "az");

  frame_info tmp_frame{};
  size_t line_counter = 0;
  while (in_csv.read_row(
      tmp_frame.time,
      tmp_frame.qw, tmp_frame.qx, tmp_frame.qy, tmp_frame.qz,
      tmp_frame.tx, tmp_frame.ty, tmp_frame.tz,
      tmp_frame.gx, tmp_frame.gy, tmp_frame.gz,
      tmp_frame.ax, tmp_frame.ay, tmp_frame.az
  )) {
    frame_database[line_counter] = vislam::data::frame(
        line_counter, {}, {},
        vislam::Vec3_t(tmp_frame.tx, tmp_frame.ty, tmp_frame.tz),
        vislam::Quat_t(tmp_frame.qw, tmp_frame.qx, tmp_frame.qy, tmp_frame.qz));
    line_counter++;
  }

  return frame_database;
}
LogPlayer_vio_dataset::landmark_database_t LogPlayer_vio_dataset::make_landmark_database(const std::string &path_to_landmark_position) {
  landmark_database_t landmark_database;
  io::CSVReader<4, io::trim_chars<>, io::no_quote_escape<' '>> in_csv(path_to_landmark_position);
  in_csv.set_header("x", "y", "z", "dummy");

  landmark_info tmp_landmark = {};
  size_t line_counter = 0;
  while (in_csv.read_row(tmp_landmark.x, tmp_landmark.y, tmp_landmark.z, tmp_landmark.dummy)) {
    landmark_database[line_counter] = vislam::data::landmark(
        line_counter, {},
        vislam::Vec3_t(tmp_landmark.x, tmp_landmark.y, tmp_landmark.z),
        false,
        true,
        true);
    line_counter++;
  }

  return landmark_database;
}
void LogPlayer_vio_dataset::add_landmark_observation(const std::string &path_to_keyframe_file,
                                                     uint64_t frame_id,
                                                     LogPlayer_vio_dataset::frame_database_t &frame_database,
                                                     LogPlayer_vio_dataset::landmark_database_t &landmark_database) {

  io::CSVReader<6, io::trim_chars<>, io::no_quote_escape<' '>> in_csv(path_to_keyframe_file);
  in_csv.set_header("lx", "ly", "lz", "dummy", "fx", "fy");

  frame_observation_info frame_observation = {};
  size_t line_counter = 0;
  while (in_csv.read_row(
      frame_observation.lx, frame_observation.ly, frame_observation.lz, frame_observation.dummy,
      frame_observation.fx, frame_observation.fy)) {

    size_t landmark_id = line_counter;

    //! Update Frame observation
    frame_database[frame_id].observingFeatureId.insert(landmark_id);
    frame_database[frame_id].observingFeaturePointInDevice[landmark_id] =
        vislam::Vec2_t(frame_observation.fx, frame_observation.fy);

    //! Update Landmark observation
    landmark_database[landmark_id].observedFrameId.insert(frame_id);

    line_counter++;
  }
}


