# Add sources
# これで ${PROJECT_NAME}というターゲットにソースが追加されるぽい
target_sources(${VSLAM_PROJECT_NAME}
        PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/DoubleSphereProjectionFactor.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/GeneralProjectionFactor.hpp
        )