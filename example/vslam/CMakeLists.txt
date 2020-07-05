set(CMAKE_C_COMPILER /usr/bin/gcc-10)
set(CMAKE_CXX_COMPILER /usr/bin/g++-10)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(example_hello example_hello.cpp)
list(APPEND EXECUTABLE_TARGETS example_hello)
add_executable(example_feature_detector example_feature_detector.cpp)
list(APPEND EXECUTABLE_TARGETS example_feature_detector)
add_executable(example_feature_detection_and_tracking example_feature_detection_and_tracking.cpp)
list(APPEND EXECUTABLE_TARGETS example_feature_detection_and_tracking)
add_executable(example_kimera_frontend_detection_and_tracking example_kimera_frontend_detection_and_tracking.cpp)
list(APPEND EXECUTABLE_TARGETS example_kimera_frontend_detection_and_tracking)
add_executable(example_kimera_backend example_kimera_backend.cpp)
list(APPEND EXECUTABLE_TARGETS example_kimera_backend)
add_executable(example_serialization example_serialization.cpp)
list(APPEND EXECUTABLE_TARGETS example_serialization)
add_executable(example_continuous_detection example_continuous_detection.cpp)
list(APPEND EXECUTABLE_TARGETS example_continuous_detection)
add_executable(example_cd_arg example_cd_arg.cpp)
list(APPEND EXECUTABLE_TARGETS example_cd_arg)


foreach (EXECUTABLE_TARGET IN LISTS EXECUTABLE_TARGETS)

    target_link_libraries(${EXECUTABLE_TARGET}
            ${VSLAM_PROJECT_NAME}
            Eigen3::Eigen
            opencv_imgcodecs
            opencv_videoio
            opencv_viz
            opencv_calib3d
            spdlog::spdlog
            fmt::fmt
            )

endforeach (EXECUTABLE_TARGET IN LISTS EXECUTABLE_TARGETS)
