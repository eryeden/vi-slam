# Add sources
# これで ${PROJECT_NAME}というターゲットにソースが追加されるぽい
target_sources(${VSLAM_PROJECT_NAME}
        PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/FeatureDetectorShiTomasi.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/FeatureDetectorShiTomasi.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/kdtree/KDTree.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/kdtree/KDTree.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/FeatureTrackerLucasKanade.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/FeatureTrackerLucasKanade.hpp
        )