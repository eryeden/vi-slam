# Add sources
# これで ${PROJECT_NAME}というターゲットにソースが追加されるぽい
target_sources(${VSLAM_PROJECT_NAME}
        PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/ThreadsafeContainer.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/ThreadsafeContainer.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/Camera.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/Camera.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/Frame.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/Frame.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/Landmark.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/Landmark.hpp
        )