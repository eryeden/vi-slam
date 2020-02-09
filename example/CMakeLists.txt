add_executable(sample1 sample1.cpp)
list(APPEND EXECUTABLE_TARGETS sample1)
add_executable(sample2 sample2.cpp)
list(APPEND EXECUTABLE_TARGETS sample2)
add_executable(sample2_cam sample2_cam.cpp)
list(APPEND EXECUTABLE_TARGETS sample2_cam)
add_executable(sample2_euroc_mav sample2_euroc_mav.cpp)
list(APPEND EXECUTABLE_TARGETS sample2_euroc_mav)
add_executable(sample2_initialize sample2_initialize.cpp)
list(APPEND EXECUTABLE_TARGETS sample2_initialize)
add_executable(sample2_pnp sample2_pnp.cpp)
list(APPEND EXECUTABLE_TARGETS sample2_pnp)
add_executable(sample2_reprojection sample2_reprojection.cpp)
list(APPEND EXECUTABLE_TARGETS sample2_reprojection)
add_executable(sample2_ba_pre sample2_ba_pre.cpp)
list(APPEND EXECUTABLE_TARGETS sample2_ba_pre)


add_executable(sample_viz sample_viz.cpp)
list(APPEND EXECUTABLE_TARGETS sample_viz)


foreach(EXECUTABLE_TARGET IN LISTS EXECUTABLE_TARGETS)

    target_link_libraries(${EXECUTABLE_TARGET}
        ${PROJECT_NAME}
        utils
        Eigen3::Eigen
        opencv_imgcodecs
        opencv_videoio
        opencv_viz
        opencv_calib3d)

endforeach(EXECUTABLE_TARGET IN LISTS EXECUTABLE_TARGETS)
