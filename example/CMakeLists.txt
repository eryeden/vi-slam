add_executable(sample1 sample1.cpp)
list(APPEND EXECUTABLE_TARGETS sample1)

foreach(EXECUTABLE_TARGET IN LISTS EXECUTABLE_TARGETS)
    target_link_libraries(${EXECUTABLE_TARGET}
        feature
        opencv_imgcodecs
        opencv_videoio)    
endforeach(EXECUTABLE_TARGET IN LISTS EXECUTABLE_TARGETS)
