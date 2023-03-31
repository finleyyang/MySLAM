file(GLOB TEST_VSLAM RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "*.[ch]pp")

add_executable(${PROJECT_NAME}_TEST_VSLAM
        ${TEST_VSLAM})

target_link_libraries(${PROJECT_NAME}_TEST_VSLAM
        ${MySLAM_LIBNAME})