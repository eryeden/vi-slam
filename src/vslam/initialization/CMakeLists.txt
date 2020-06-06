# Add sources
# これで ${PROJECT_NAME}というターゲットにソースが追加されるぽい
target_sources(${VSLAM_PROJECT_NAME}
        PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/Initialization.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/Initialization.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/PoseInitialization.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/PoseInitialization.hpp
        )