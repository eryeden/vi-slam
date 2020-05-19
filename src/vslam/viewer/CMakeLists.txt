# Add sources
# これで ${PROJECT_NAME}というターゲットにソースが追加されるぽい
target_sources(${VSLAM_PROJECT_NAME}
        PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/Viewer.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/Viewer.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/ViewerViz.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/ViewerViz.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/ViewerPrimitives.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/ViewerPrimitives.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/ThreadsafeQueue.hpp
        )