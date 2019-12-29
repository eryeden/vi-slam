#include <iostream>
#include <string>
#include <algorithm>
#include <numeric>
#include <pthread.h>
#include <signal.h>
#include <time.h>

#include <sstream>
#include <iomanip>

#include <chrono>
#include <deque>
#include <tuple>

#include "csv.h"

#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>

class LogPlayer_extended {
public:
    using log_pack = std::tuple<
        uint64_t,   //index
        uint64_t,   // save time
        uint64_t,   // image capture time
        uint64_t,   // pose capture time
        double,     // utm_x
        double,     // utm_y
        double,     // heading
        double,     // utm_x_latest
        double,     // utm_y_latest
        double,     // heading_latest
        std::string // path to image
        >;

    LogPlayer_extended(
        const std::string& path_to_log_dir,
        const double log_sample_time_step)
        : pathToLogDir(path_to_log_dir),
          logSampleTimeStep(static_cast<uint64_t>(log_sample_time_step * 1e9)),
          loggerClock(0) {
        logsStored = log_parser(pathToLogDir + "/log.csv");
        printf("%s : %s Log loaded %ld lines\n", __func__, path_to_log_dir.c_str(), logsStored.size());
        loggerClock = std::get<1>(logsStored[0]);
        lastIndex = 0;
    }



    bool get_frame_by_index(cv::Mat& img, double& timestamp_in_sec, const uint64_t frame_index) {
        if (!(frame_index < logsStored.size())) {
            return false;
        }

        std::string path_to_image = pathToLogDir + "/" + basename(std::get<10>(logsStored[frame_index]));

        cv::Mat loaded_img = cv::imread(path_to_image, -1);

        loaded_img.copyTo(img);

        timestamp_in_sec = static_cast<double>(std::get<3>(logsStored[frame_index])) * 1e-9;

        return true;
    }

    int32_t get_frame_size() {
        return logsStored.size();
    }

private:
    std::string GetPathToImageByIndexAndZeroFillNum(
        const std::string& path_to_image_prefix_,
        const std::string& path_to_image_postfix_,
        int32_t zero_fill_qty_,
        int32_t image_index_,
        const std::string& image_type) {
        std::ostringstream sout;
        sout << std::setfill('0') << std::setw(zero_fill_qty_) << image_index_;
        return path_to_image_prefix_ + sout.str() + path_to_image_postfix_ + image_type;
    }

    std::string basename(const std::string& path) {
        return path.substr(path.find_last_of('/') + 1);
    }

    std::vector<log_pack> log_parser(const std::string& path_to_csv) {
        std::vector<log_pack> logs;

        //                std::string csv_line = std::to_string(image_index);
        //                csv_line += std::string(",") + std::to_string(cap_time_image);
        //                csv_line += std::string(",") + std::to_string(cap_time_pose);
        //                csv_line += std::string(",") + std::to_string(pose.position_north_x_meter());
        //                csv_line += std::string(",") + std::to_string(pose.position_east_y_meter());
        //                csv_line += std::string(",") + std::to_string(pose.heading_rad());
        //                csv_line += std::string(",") + path_to_save_captured_image;
        //                csv_line += std::string("\n");

        io::CSVReader<11> in_csv(path_to_csv);
        in_csv.set_header("index", "time_save", "time_image_capture", "time_pose_capture", "utm_x", "utm_y", "heading", "utm_x_latest", "utm_y_latest", "heading_latest",
                          "path_to_captured_image");

        log_pack tmp_log_pack;
        while (in_csv.read_row(std::get<0>(tmp_log_pack),
                               std::get<1>(tmp_log_pack),
                               std::get<2>(tmp_log_pack),
                               std::get<3>(tmp_log_pack),
                               std::get<4>(tmp_log_pack),
                               std::get<5>(tmp_log_pack),
                               std::get<6>(tmp_log_pack),
                               std::get<7>(tmp_log_pack),
                               std::get<8>(tmp_log_pack),
                               std::get<9>(tmp_log_pack),
                               std::get<10>(tmp_log_pack))) {
            logs.emplace_back(tmp_log_pack);
        }
        return logs;
    }

    std::string pathToLogDir;
    uint64_t logSampleTimeStep;

    std::vector<log_pack> logsStored;

    uint64_t loggerClock;
    uint64_t lastIndex;
};