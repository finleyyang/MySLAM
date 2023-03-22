file(GLOB TEST_ORBextractor RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "*.[ch]pp")

add_executable(${PROJECT_NAME}_TEST_ORBectractor
        ${TEST_ORBextractor})

target_link_libraries(${PROJECT_NAME}_TEST_ORBectractor
        ${MySLAM_LIBNAME})