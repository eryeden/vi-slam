add_executable(example_gtsam_test1 example_gtsam_test1.cpp)
list(APPEND EXECUTABLE_TARGETS example_gtsam_test1)
add_executable(example_gtsam_odometry example_gtsam_odometry.cpp)
list(APPEND EXECUTABLE_TARGETS example_gtsam_odometry)
add_executable(example_gtsam_robot_localization example_gtsam_robot_localization.cpp)
list(APPEND EXECUTABLE_TARGETS example_gtsam_robot_localization)
add_executable(example_gtsam_landmark_based_slam example_gtsam_landmark_based_slam.cpp)
list(APPEND EXECUTABLE_TARGETS example_gtsam_landmark_based_slam)


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
