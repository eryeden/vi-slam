//
// Created by ery on 2020/06/10.
//

#include "type_defines.hpp"

namespace vslam::dataoutput {

class DataOutputBase {
 public:
  DataOutputBase() = default;
  virtual ~DataOutputBase() = default;
  virtual void SavePose(double timestamp_sec, vslam::Pose_t pose) = 0;

 private:
};

}  // namespace vslam::dataoutput