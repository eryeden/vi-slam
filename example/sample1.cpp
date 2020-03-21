#include <iostream>
#include <opencv2/opencv.hpp>
#include "feature_test.hpp"

#include "log_util.h"

int main() {

  LogPlayer_extended lpe("/home/ery/Devel/tmp/assets/20191219_2/20191219_24", 0);

  for (size_t i = 0; i < lpe.get_frame_size(); i++) {
    cv::Mat img;
    double timestamp;
    lpe.get_frame_by_index(img, timestamp, i);

    cv::imshow("Test", img);
    cv::waitKey(1);
  }
}