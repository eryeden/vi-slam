# Add sources
# これで ${PROJECT_NAME}というターゲットにソースが追加されるぽい
target_sources(${VSLAM_PROJECT_NAME}
        PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/DataProviderBase.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/DataProviderBase.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/EurocKimeraDataProvider.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/EurocKimeraDataProvider.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/EurocKimeraDataProviderRadialTangentialCameraModel.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/EurocKimeraDataProviderRadialTangentialCameraModel.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/DataOutputBase.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/DataOutputBase.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/TumDataOutput.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/TumDataOutput.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/csv.h
        )