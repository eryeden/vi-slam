#add_library(utils SHARED)
#
## Include directories
## ここでInclude diretoryを追加しておけば、target_link_librariesやったときに参照されるようになる
#target_include_directories(utils
#        PUBLIC
#        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src/utils/loader>
#        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src/utils/>)
#
## Link to required libraries
#target_link_libraries(utils
#        PUBLIC
#        Threads::Threads
#        yaml-cpp
#        opencv_core
#        opencv_features2d
#        opencv_calib3d)

add_subdirectory(loader)

