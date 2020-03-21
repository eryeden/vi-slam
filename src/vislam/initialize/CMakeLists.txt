# Add sources
# これで ${PROJECT_NAME}というターゲットにソースが追加されるぽい
target_sources(${PROJECT_NAME}
        PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/initializer.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/initializer.hpp
        )