# Add sources
# これで ${PROJECT_NAME}というターゲットにソースが追加されるぽい
target_sources(${PROJECT_NAME}
        PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/feature_in_frame.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/feature_in_frame.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/frame.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/frame.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/landmark.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/landmark.hpp
        )