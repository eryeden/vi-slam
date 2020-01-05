#include <iostream>
#include <opencv2/opencv.hpp>
#include "dense_feature_extructor.hpp"

int main()
{

    // LogPlayer_extended lpe("/home/ery/Devel/tmp/assets/20191219_2/20191219_24", 0);

    dense_feature_extructor dfe;
    // dfe.run_extruction("/home/ery/assets/20191219_2/20191219_31");
    // dfe.run_extruction("/home/ery/assets/20191219_1/20191219_2");
    dfe.run_extruction("/home/ery/assets/20191115/20191115_40_2m_track");

    // dfe.run_extruction("/home/ery/Devel/tmp/assets/20191219_2/20191219_31");
    // dfe.run_extruction_cam("/dev/video0", 1.0);
    // dfe.run_extruction_cam("/home/ery/Devel/tmp/assets/IMG_5144.MOV", 1.0 / 2.0);
}