file(GLOB TEST_FrameMatch RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "*.[ch]pp")

add_executable(${PROJECT_NAME}_TEST_FrameMatch
        ${TEST_FrameMatch})

target_link_libraries(${PROJECT_NAME}_TEST_FrameMatch
        ${MySLAM_LIBNAME})