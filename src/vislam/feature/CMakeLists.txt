# Add sources
# これで ${PROJECT_NAME}というターゲットにソースが追加されるぽい
target_sources(${PROJECT_NAME}
            PRIVATE               
            ${CMAKE_CURRENT_SOURCE_DIR}/feature_test.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/feature_test.hpp)