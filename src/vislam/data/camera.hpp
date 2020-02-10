//
// Created by anudev on 2020/02/10.
//

#pragma once

#include "type_defines.hpp"

namespace vislam::data {

    struct camera {

    public:
        camera(uint8_t id, uint32_t width, uint32_t height, double fps,
               double fx,double fy,double cx,double cy,
               double k1,double k2,double p1, double p2,double k3);
        camera();

        Mat33_t get_intrinsic_matrix() const;
        std::vector<double> get_distortion_parameters() const ;

        void set_camera_intrinsic_parameter(const Mat33_t & intrinsic);
        void set_camera_distortion_parameter(const std::vector<double> & distortion_parameter);

        //! camera id
        uint8_t id;

        //! width of image
        uint32_t width;
        //! height of image
        uint32_t height;

        //! frame rate of image
        double fps;

        //! pinhole params
        double fx;
        double fy;
        double cx;
        double cy;
        double fx_inv;
        double fy_inv;

        //! distortion params
        double k1;
        double k2;
        double p1;
        double p2;
        double k3;

    private:

    };


}