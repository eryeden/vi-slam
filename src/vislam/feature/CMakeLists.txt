# Add sources
# これで ${PROJECT_NAME}というターゲットにソースが追加されるぽい
target_sources(${VISLAM_PROJECT_NAME}
        PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/feature_test.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/feature_test.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/dense_feature_extructor_proto.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/dense_feature_extructor_proto.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/dense_feature_extructor.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/dense_feature_utils.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/dense_feature_extructor.hpp
        )