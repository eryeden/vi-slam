//
// Created by ery on 2020/05/02.
//

#pragma once

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>

namespace vslam::utility {

cv::Mat ConvertEigenMatToCVMat(const Eigen::MatrixXd& eigen_mat);
cv::Mat ConvertEigenMatToCVMat(const Eigen::Matrix3d& eigen_mat);

}  // namespace vslam::utility