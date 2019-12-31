add_executable(sample1 sample1.cpp)
list(APPEND EXECUTABLE_TARGETS sample1)
add_executable(sample2 sample2.cpp)
list(APPEND EXECUTABLE_TARGETS sample2)


foreach(EXECUTABLE_TARGET IN LISTS EXECUTABLE_TARGETS)

    target_link_libraries(${EXECUTABLE_TARGET}
        ${PROJECT_NAME}
        utils
        opencv_imgcodecs
        opencv_videoio)

endforeach(EXECUTABLE_TARGET IN LISTS EXECUTABLE_TARGETS)
