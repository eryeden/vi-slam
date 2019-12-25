# OpenMP
set(USE_OPENMP OFF CACHE BOOL "Use OpenMP")
if(USE_OPENMP)
    find_package(OpenMP)
    if(OpenMP_FOUND)
        set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
        target_compile_definitions(${PROJECT_NAME} PUBLIC USE_OPENMP)
        message(STATUS "OpenMP: ENABLED")
    else()
        message(STATUS "OpenMP: DISABLED")
    endif()
else()
    message(STATUS "OpenMP: DISABLED")
endif()



add_subdirectory(data)
add_subdirectory(feature)

