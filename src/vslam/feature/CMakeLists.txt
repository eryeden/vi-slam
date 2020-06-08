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
        ${CMAKE_CURRENT_SOURCE_DIR}/FeatureTrackerLSSDLucasKanade.cpp

        # Codes from basalt project
        ${CMAKE_CURRENT_SOURCE_DIR}/LSSDOpticalFlow/frame_to_frame_optical_flow.h
        ${CMAKE_CURRENT_SOURCE_DIR}/LSSDOpticalFlow/optical_flow.h
        ${CMAKE_CURRENT_SOURCE_DIR}/LSSDOpticalFlow/optical_flow.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/LSSDOpticalFlow/patch.h
        ${CMAKE_CURRENT_SOURCE_DIR}/LSSDOpticalFlow/patch_optical_flow.h
        ${CMAKE_CURRENT_SOURCE_DIR}/LSSDOpticalFlow/patterns.h
        ${CMAKE_CURRENT_SOURCE_DIR}/LSSDOpticalFlow/utils/common_types.h
        ${CMAKE_CURRENT_SOURCE_DIR}/LSSDOpticalFlow/utils/keypoints.h
        ${CMAKE_CURRENT_SOURCE_DIR}/LSSDOpticalFlow/utils/vio_config.h
        ${CMAKE_CURRENT_SOURCE_DIR}/LSSDOpticalFlow/utils/keypoints.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/LSSDOpticalFlow/utils/vio_config.cpp
        )

add_subdirectory(NonMaximumSuppression/anms)