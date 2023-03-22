cmake_minimum_required(VERSION 3.24)
project(MySLAM)

file(GLOB_RECURSE INCLUDES_ALL RELATIVE "${PROJECT_SOURCE_DIR}" "include/*.h")
file(GLOB_RECURSE SOURCES_ALL RELATIVE "${PROJECT_SOURCE_DIR}" "src/*.[ch]pp")
string(TOLOWER ${PROJECT_NAME} PROJECT_NAME_LOWER)
set(MySLAM_LIBNAME ${PROJECT_NAME_LOWER} CACHE INTERNAL "")

add_library(${MySLAM_LIBNAME} ${INCLUDES_ALL} ${SOURCES_ALL})
source_group(TREE "${PROJECT_SOURCE_DIR}/include"
        PREFIX "Header Files"
        FILES ${INCLUDES_ALL}
        )
source_group(TREE "${PROJECT_SOURCE_DIR}/src"
        PREFIX "Source Files"
        FILES ${SOURCE_ALL}
        )
target_include_directories(${MySLAM_LIBNAME}
        PUBLIC
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src>
)
set(CMAKE_CXX_STANDARD 17)


find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)
find_package(Pangolin REQUIRED)
find_package(spdlog REQUIRED)


include_directories(
        ${EIGEN3_INCLUDE_DIR}
        ${OpenCV_INCLUDE_DIRS}
        ${CERES_INCLUDE_DIRS}
        ${Pangolin_INCLUDE_DIRS}
        ${spdlog_INCLUDE_DIRS}
)

target_link_libraries(${MySLAM_LIBNAME}
        ${OpenCV_LIBS}
        ${EIGEN3_LIBS}
        ${CERES_LIBS}
        ${Pangolin_LIBS}
        ${spdlog_LIBS}
        )
add_subdirectory(test)

string(TOUPPER ${PROJECT_NAME} PROJECT_NAME_UPPER)

install(TARGETS
        ${MySLAM_LIBNAME}
        EXPORT ${PROJECT_NAME_LOWER}-config
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include
        )

install(DIRECTORY ${PROJECT_SOURCE_DIR}/include
        DESTINATION ${CMAKE_INSTALL_PREFIX}
        FILES_MATCHING PATTERN "*.h*"
        )

install(EXPORT ${PROJECT_NAME_LOWER}-config DESTINATION cmake)