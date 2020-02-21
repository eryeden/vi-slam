# ----- Create vislam library -----

add_library(${PROJECT_NAME}
            SHARED
            ${CMAKE_CURRENT_SOURCE_DIR}/type_defines.hpp
            )

# OpenMP
set(USE_OPENMP ON CACHE BOOL "Use OpenMP")
if(USE_OPENMP)
    find_package(OpenMP)
    if(OpenMP_FOUND)
        set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
        target_compile_definitions(${PROJECT_NAME} PUBLIC USE_OPENMP)
        message(STATUS "OpenMP: ENABLED")
    else()
        message(STATUS "OpenMP: DISABLED")
    endif()
else()
    message(STATUS "OpenMP: DISABLED")
endif()

# Include directories
# ここでInclude diretoryを追加しておけば、target_link_librariesやったときに参照されるようになる
target_include_directories(${PROJECT_NAME}
        PUBLIC
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src/vislam/>
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src/vislam/feature>
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src/vislam/data>
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src/vislam/initialize>
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src/vislam/bundle_adjustment>)

# Link to required libraries
target_link_libraries(${PROJECT_NAME}
                    PUBLIC
                    utils
                    Eigen3::Eigen
                    yaml-cpp
                    opencv_core
                    opencv_highgui
                    opencv_imgproc
                    opencv_features2d
                    opencv_xfeatures2d
                    opencv_calib3d)



add_subdirectory(data)
add_subdirectory(feature)
add_subdirectory(initialize)
add_subdirectory(bundle_adjustment)
