//
// Created by ery on 2020/06/12.
//

#include "LogDataOutput.hpp"
vslam::dataoutput::LogDataOutput::LogDataOutput(
    const std::string& path_to_save_dir)
:path_to_save_dir_(path_to_save_dir)
{

  /**
   * @brief frame dump dirの生成も必要
   */
  path_to_frame_dump_dir = path_to_save_dir + "/frames/";

}
void vslam::dataoutput::LogDataOutput::DumpInternals(
    database_index_t frame_number,
    const vslam::data::InternalMaterials& internal_materials)
{

}
