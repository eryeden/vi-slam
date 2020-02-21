# Add sources
# これで ${PROJECT_NAME}というターゲットにソースが追加されるぽい
target_sources(${PROJECT_NAME}
        PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/ba_pre.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/ba_pre.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/geometry.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/geometry.hpp
        )