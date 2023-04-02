file(GLOB TEST_DBoW RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "*.[ch]pp")

add_executable(${PROJECT_NAME}_TEST_DBoW
        ${TEST_DBoW})

target_link_libraries(${PROJECT_NAME}_TEST_DBoW
        ${MySLAM_LIBNAME})