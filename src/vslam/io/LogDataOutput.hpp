//
// Created by ery on 2020/06/12.
//

#pragma once

#include "type_defines.hpp"
#include "Internals.hpp"

namespace vslam::dataoutput {

/**
 * @brief Logデータ関係をまとめるクラスにする予定
 * @details
 * コンセプト：MlFlowで各種評価、実験管理を行えるようなPyhtonから絵使えるLog出力を行う。
 * そのため、以下機能を実装する
 * - 実行時、パラメータ指定ができる
 * - 指定したDirectoryにすべての結果をまとめて出力できる <= こちら側を実装する
 *
 *
 */
class LogDataOutput{
 public:
  /**
   * @brief Frame Internalsの保存ディレクトリ作成
   * @param path_to_save_dir
   */
  LogDataOutput(const std::string& path_to_save_dir);


  /**
   * @brief Frame Internalsを１ファイルとして保存する
   * @param internal_materials
   */
  void DumpInternals(
      database_index_t frame_number,
      const data::InternalMaterials& internal_materials);

 private:

  std::string path_to_save_dir_;
  std::string path_to_frame_dump_dir;
};

}