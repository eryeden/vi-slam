add_library(feature
    feature_test.cpp
    feature_test.hpp)

target_link_libraries(feature
    Eigen3::Eigen
    yaml-cpp
    opencv_core
    opencv_features2d
    opencv_calib3d)