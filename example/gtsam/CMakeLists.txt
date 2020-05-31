add_executable(example_gtsam_test1 example_gtsam_test1.cpp)
list(APPEND EXECUTABLE_TARGETS example_gtsam_test1)
add_executable(example_gtsam_odometry example_gtsam_odometry.cpp)
list(APPEND EXECUTABLE_TARGETS example_gtsam_odometry)
add_executable(example_gtsam_robot_localization example_gtsam_robot_localization.cpp)
list(APPEND EXECUTABLE_TARGETS example_gtsam_robot_localization)
add_executable(example_gtsam_landmark_based_slam example_gtsam_landmark_based_slam.cpp)
list(APPEND EXECUTABLE_TARGETS example_gtsam_landmark_based_slam)
add_executable(example_gtsam_sfm1 example_gtsam_sfm1.cpp)
list(APPEND EXECUTABLE_TARGETS example_gtsam_sfm1)
add_executable(example_gtsam_sfm_smart_factor example_gtsam_sfm_smart_factor.cpp)
list(APPEND EXECUTABLE_TARGETS example_gtsam_sfm_smart_factor)
add_executable(example_gtsam_isam2 example_gtsam_isam2.cpp)
list(APPEND EXECUTABLE_TARGETS example_gtsam_isam2)
add_executable(example_gtsam_ds_camera_model example_gtsam_ds_camera_model.cpp)
list(APPEND EXECUTABLE_TARGETS example_gtsam_ds_camera_model)

add_executable(example_ds_camera_model_projection_test example_ds_camera_model_projection_test.cpp)
list(APPEND EXECUTABLE_TARGETS example_ds_camera_model_projection_test)

add_executable(example_pangolin example_pangolin.cpp)
list(APPEND EXECUTABLE_TARGETS example_pangolin)
add_executable(example_viewer_test1 example_viewer_test1.cpp)
list(APPEND EXECUTABLE_TARGETS example_viewer_test1)
add_executable(example_viewer_covariance_test example_viewer_covariance_test.cpp)
list(APPEND EXECUTABLE_TARGETS example_viewer_covariance_test)


foreach (EXECUTABLE_TARGET IN LISTS EXECUTABLE_TARGETS)

    target_link_libraries(${EXECUTABLE_TARGET}
            ${VSLAM_PROJECT_NAME}
            spdlog::spdlog
            fmt::fmt
            )

endforeach (EXECUTABLE_TARGET IN LISTS EXECUTABLE_TARGETS)
