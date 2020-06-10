//
// Created by ery on 2020/06/10.
//

#include <fstream>
#include <iostream>
#include <string>

#include "DataOutputBase.hpp"

namespace vslam::dataoutput {

class TumDataOutput : public DataOutputBase {
 public:
  explicit TumDataOutput(const std::string& path_to_save);

  void SavePose(double timestamp_sec, vslam::Pose_t pose) override;

 private:
  std::string path_to_save_;
  std::ofstream output_stream_;
};

}  // namespace vslam::dataoutput