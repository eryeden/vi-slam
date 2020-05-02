//
// Created by ery on 2020/05/02.
//

#include "OpenCVUtilities.hpp"

#include <opencv2/core/eigen.hpp>

using namespace vslam;
using namespace vslam::utility;

cv::Mat vslam::utility::ConvertEigenMatToCVMat(
    const Eigen::MatrixXd& eigen_mat) {
  cv::Mat out(eigen_mat.rows(), eigen_mat.cols(), CV_64FC1);
  cv::eigen2cv(eigen_mat, out);
  return out;
}
cv::Mat vslam::utility::ConvertEigenMatToCVMat(
    const Eigen::Matrix3d& eigen_mat) {
  cv::Mat out(3, 3, CV_64FC1);
  cv::eigen2cv(eigen_mat, out);
  return out;
}
