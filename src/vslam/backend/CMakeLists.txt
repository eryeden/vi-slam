# Add sources
# これで ${PROJECT_NAME}というターゲットにソースが追加されるぽい
target_sources(${VSLAM_PROJECT_NAME}
        PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/BackendBase.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/BackendBase.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/iSAM2Backend.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/iSAM2BackendParameter.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/iSAM2Backend.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/ContinuousDetectorBackend.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/ContinuousDetectorBackendParameter.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/ContinuousDetectorBackend.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/BackendUtilities.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/BackendUtilities.hpp
        )