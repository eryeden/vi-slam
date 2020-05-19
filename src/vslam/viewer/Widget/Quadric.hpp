//
// Created by ery on 2020/05/19.
//

#pragma once

#include <iostream>
#include <opencv2/viz.hpp>
#include <opencv2/viz/widget_accessor.hpp>

#include "type_defines.hpp"

/*
 * OpenCV VizのカスタムWidgetをつくる。下リンクに沿って作った。
 * https://docs.opencv.org/master/d2/d64/tutorial_creating_widgets.html
 *
 * SuperQuadric:
 * http://www.echna.ne.jp/~bunden/suprelips.html
 *
 * VTK:
 * https://lorensen.github.io/VTKExamples/site/Cxx/ImplicitFunctions/ImplicitQuadric/
 *
 *
 *
 */

namespace vslam::viewer {

class WQuadric : public cv::viz::Widget3D {
 public:
  WQuadric(const std::array<double, 10>& quadric_coefficients,
           const cv::viz::Color& color = cv::viz::Color::white());
  WQuadric(const vslam::Vec3_t& scale,
           const cv::viz::Color& color = cv::viz::Color::white());
};

class WTriangleSample : public cv::viz::Widget3D {
 public:
  WTriangleSample(const cv::Point3f& pt1,
                  const cv::Point3f& pt2,
                  const cv::Point3f& pt3,
                  const cv::viz::Color& color = cv::viz::Color::white());
};

}  // namespace vslam::viewer