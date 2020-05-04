# Add sources
# これで ${PROJECT_NAME}というターゲットにソースが追加されるぽい
target_sources(${VSLAM_PROJECT_NAME}
        PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/FeatureDetector.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/FeatureDetector.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/kdtree/KDTree.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/kdtree/KDTree.cpp
        )