# Add sources
# これで ${PROJECT_NAME}というターゲットにソースが追加されるぽい
target_sources(${VSLAM_PROJECT_NAME}
        PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/Frontend.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/Frontend.hpp
        )