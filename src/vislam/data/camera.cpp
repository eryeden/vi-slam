//
// Created by anudev on 2020/02/10.
//

#include "camera.hpp"

vislam::data::camera::camera(uint8_t id_, uint32_t width_, uint32_t height_,
                             double fps_,
                             double fx_, double fy_,
                             double cx_,double cy_,
                             double k1_, double k2_, double p1_, double p2_, double k3_)
        :id(id_), width(width_), height(height_),
         fps(fps_), fx(fx_), fy(fy_),
         cx(cx_), cy(cy_),
         k1(k1_), k2(k2_), p1(p1_), p2(p2_), k3(k3_),
         fx_inv(1.0/fx_),fy_inv(1.0/fy_)
{
;
}

vislam::data::camera::camera()
:camera(std::numeric_limits<uint8_t>::max(),
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0)
{
;
}


vislam::Mat33_t vislam::data::camera::get_intrinsic_matrix(){
    Mat33_t out;
    out   << fx, 0, cx, 0, fy, cy, 0,0,1.0;
    return out;
}
std::vector<double> vislam::data::camera::get_distortion_parameters(){
    return {k1, k2, p1, p2, k3};
}

void vislam::data::camera::set_camera_intrinsic_parameter(const vislam::Mat33_t &intrinsic) {
    fx = intrinsic(0,0);
    fy = intrinsic(1,1);
    cx = intrinsic(2, 0);
    cy = intrinsic(2,1);
    fx_inv = 1.0/fx;
    fy_inv = 1.0/fy;
}

void vislam::data::camera::set_camera_distortion_parameter(const std::vector<double> &distortion_parameter) {
    k1=distortion_parameter[0];
    k2=distortion_parameter[1];
    p1=distortion_parameter[2];
    p2=distortion_parameter[3];
    k3=distortion_parameter[4];
}
