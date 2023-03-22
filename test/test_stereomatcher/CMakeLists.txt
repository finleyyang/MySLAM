file(GLOB TEST_StereoMatcher RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "*.[ch]pp")

add_executable(${PROJECT_NAME}_TEST_StereoMatcher
        ${TEST_StereoMatcher})

target_link_libraries(${PROJECT_NAME}_TEST_StereoMatcher
        ${MySLAM_LIBNAME})