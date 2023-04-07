file(GLOB TEST_Optim RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "*.[ch]pp")

add_executable(${PROJECT_NAME}_TEST_Optim
        ${TEST_Optim})

target_link_libraries(${PROJECT_NAME}_TEST_Optim
        ${MySLAM_LIBNAME})