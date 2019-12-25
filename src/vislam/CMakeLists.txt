add_subdirectory(data)
add_subdirectory(feature)

target_link_libraries(${PROJECT_NAME}
        ${OpenCV_LIBS}
        ${YAMLCPP_LIBRARIES})