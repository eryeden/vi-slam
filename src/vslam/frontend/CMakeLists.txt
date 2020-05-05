# Add sources
# これで ${PROJECT_NAME}というターゲットにソースが追加されるぽい
target_sources(${VSLAM_PROJECT_NAME}
        PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/FrontendBase.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/FrontendBase.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/KimeraFrontend.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/KimeraFrontend.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/Verification.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/Verification.hpp
        )