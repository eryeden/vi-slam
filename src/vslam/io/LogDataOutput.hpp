//
// Created by ery on 2020/06/12.
//

#pragma once

#include <ostream>

#include "FeatureDetectorANMS.hpp"
#include "FeatureTrackerLSSDLucasKanade.hpp"
#include "Internals.hpp"
#include "KimeraFrontend.hpp"
#include "Serializations.hpp"
#include "Verification.hpp"
#include "fmt/format.h"
#include "iSAM2Backend.hpp"
#include "type_defines.hpp"
#include "EurocKimeraDataProvider.hpp"
#include "KittiKimeraDataProvider.hpp"

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
class LogDataOutput {
 public:
  /**
   * @brief Frame Internalsの保存ディレクトリ作成
   * @param path_to_save_dir
   */
  explicit LogDataOutput(const std::string& path_to_save_dir);

  /**
   * @brief Dump
   */
  template <typename T>
  void Dump(const T& parameter, const std::string& filename) {
    std::string path_to_dump_file = path_to_save_dir_ + "/" + filename;
    std::ofstream output_stream(path_to_dump_file, std::ios::out);
    {
      cereal::JSONOutputArchive json_output_archive(output_stream);
      json_output_archive(parameter);
    }
  }

  /**
   * @brief Frame Internalsを１ファイルとして保存する
   * @param frame_number
   * @param internal_materials
   */
  void Dump(database_index_t frame_number,
            const data::InternalMaterials& internal_materials) {
    std::string path_to_internal_from_log_root =
        fmt::format("{}/frame_{}.json",
                    path_to_frame_dump_dir_from_log_root_,
                    frame_number);
    Dump<data::InternalMaterials>(internal_materials,
                                  path_to_internal_from_log_root);
  }

  /**
   * @brief Feature trackerのパラメータをDump
   * @param parameter
   */
  void Dump(
      const feature::FeatureTrackerLSSDLucasKanade::Parameter& parameter,
      const std::string& filename = "FeatureTrackerLSSDLucasKanade.json") {
    Dump<feature::FeatureTrackerLSSDLucasKanade::Parameter>(parameter,
                                                            filename);
  }
  /**
   * @brief Feature detectorのパラメータのDump
   * @param parameter
   */
  void Dump(const vslam::feature::FeatureDetectorANMS::Parameter& parameter,
            const std::string& filename = "FeatureDetectorANMS.json") {
    Dump<vslam::feature::FeatureDetectorANMS::Parameter>(parameter, filename);
  }
  /**
   * @brief FrontendのパラメータのDump
   * @param parameter
   */
  void Dump(const frontend::KimeraFrontend::Parameter& parameter,
            const std::string& filename = "KimeraFrontend.json") {
    Dump<frontend::KimeraFrontend::Parameter>(parameter, filename);
  }
  /**
   * @brief BackendのパラメータのDump
   * @param parameter
   */
  void Dump(const backend::iSAM2Backend::Parameter& parameter,
            const std::string& filename = "iSAM2Backend.json") {
    Dump<backend::iSAM2Backend::Parameter>(parameter, filename);
  }

  void Dump(
      const verification::FeatureVerification5PointRANSAC::Parameter& parameter,
      const std::string& filename = "FeatureVerification5PointRANSAC.json") {
    Dump<verification::FeatureVerification5PointRANSAC::Parameter>(parameter,
                                                                   filename);
  }

  void Dump(
      const dataprovider::EurocKimeraDataProvider::Parameter& parameter,
      const std::string& filename = "EurocKimeraDataProvider.json") {
    Dump<dataprovider::EurocKimeraDataProvider::Parameter>(parameter,
                                                           filename);
  }

  void Dump(
      const dataprovider::KittiKimeraDataProvider::Parameter& parameter,
      const std::string& filename = "KittiKimeraDataProvider.json") {
    Dump<dataprovider::KittiKimeraDataProvider::Parameter>(parameter,
                                                           filename);
  }

 private:
  std::string path_to_save_dir_;
  std::string path_to_frame_dump_dir_from_log_root_;
};

}