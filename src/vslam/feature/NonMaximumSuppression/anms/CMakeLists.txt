### Add source code for IDEs

target_sources(${VSLAM_PROJECT_NAME}
        PRIVATE
        "${CMAKE_CURRENT_LIST_DIR}/nanoflann.hpp"
        "${CMAKE_CURRENT_LIST_DIR}/range-tree/lrtypes.h"
        "${CMAKE_CURRENT_LIST_DIR}/range-tree/ranget.h"
        "${CMAKE_CURRENT_LIST_DIR}/anms.h"
        "${CMAKE_CURRENT_LIST_DIR}/anms.cpp"
        )
