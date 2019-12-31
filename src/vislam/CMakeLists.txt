# ----- Create vislam library -----

add_library(${PROJECT_NAME}
            SHARED)

# Include directories
# ここでInclude diretoryを追加しておけば、target_link_librariesやったときに参照されるようになる
target_include_directories(${PROJECT_NAME}
                           PUBLIC
                           $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src/vislam/>
                           $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src/vislam/feature>
                           $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src/vislam/data>)

# Link to required libraries
target_link_libraries(${PROJECT_NAME}
                      PUBLIC
                      utils
                      yaml-cpp
                      opencv_core
                      opencv_features2d
                      opencv_calib3d)

add_subdirectory(data)
add_subdirectory(feature)
