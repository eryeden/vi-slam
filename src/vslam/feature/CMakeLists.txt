# Add sources
# これで ${PROJECT_NAME}というターゲットにソースが追加されるぽい
target_sources(${VSLAM_PROJECT_NAME}
        PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/FeatureDetectorShiTomasiBucketing.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/FeatureDetectorShiTomasiBucketing.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/FeatureDetectorBase.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/FeatureDetectorBase.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/FeatureDetectorANMS.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/FeatureDetectorANMS.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/NonMaximumSuppression/NonMaximumSuppression.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/NonMaximumSuppression/NonMaximumSuppression.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/kdtree/KDTree.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/kdtree/KDTree.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/FeatureTrackerLucasKanade.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/FeatureTrackerLucasKanade.hpp
        )

add_subdirectory(NonMaximumSuppression/anms)