cmake_minimum_required(VERSION 3.15)
project(PathFinder)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(BUILD_EXAMPLES "Build examples" OFF)
option(BUILD_SHARED_LIBS "Build using shared libraries" OFF)


if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g -Wall -Wextra -Wpedantic -Wno-unused-parameter")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

add_library(
    ${PROJECT_NAME}
    src/a_star.cpp
    src/rrt_star.cpp
    src/rrt_star2.cpp

    src/point.cpp
    src/polygon.cpp
    
    src/tree.cpp
    src/tree_node.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_LIST_DIR})

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/bin)

if(BUILD_EXAMPLES)
    add_executable(
      ${PROJECT_NAME}RRTStar
        tests/test_rrt_star.cpp
    )
    target_link_libraries(${PROJECT_NAME}RRTStar ${PROJECT_NAME})

    add_executable(
      ${PROJECT_NAME}RRTStar2
        tests/test_rrt_star2.cpp
    )
    target_link_libraries(${PROJECT_NAME}RRTStar2 ${PROJECT_NAME})

    add_executable(
      ${PROJECT_NAME}AStar
        tests/test_a_star.cpp
    )
    target_link_libraries(${PROJECT_NAME}AStar ${PROJECT_NAME})
endif()
