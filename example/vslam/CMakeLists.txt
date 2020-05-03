add_executable(example_hello example_hello.cpp)
list(APPEND EXECUTABLE_TARGETS example_hello)
add_executable(example_feature_detector example_feature_detector.cpp)
list(APPEND EXECUTABLE_TARGETS example_feature_detector)

foreach (EXECUTABLE_TARGET IN LISTS EXECUTABLE_TARGETS)

    target_link_libraries(${EXECUTABLE_TARGET}
            ${VSLAM_PROJECT_NAME}
            Eigen3::Eigen
            opencv_imgcodecs
            opencv_videoio
            opencv_viz
            opencv_calib3d)

endforeach (EXECUTABLE_TARGET IN LISTS EXECUTABLE_TARGETS)
