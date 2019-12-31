#include <iostream>
#include <opencv2/opencv.hpp>
#include "dense_feature_extructor.hpp"

int main()
{

    // LogPlayer_extended lpe("/home/ery/Devel/tmp/assets/20191219_2/20191219_24", 0);

    dense_feature_extructor dfe;
    dfe.run_extruction("/home/ery/assets/20191219_2/20191219_31");
}