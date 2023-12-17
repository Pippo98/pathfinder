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

    src/point.cpp
    src/polygon.cpp
    
    src/tree.cpp
    src/tree_node.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_LIST_DIR})

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/bin)

if(BUILD_EXAMPLES)
    add_executable(
        ${PROJECT_NAME}Test1
        tests/test_1.cpp
    )
    target_link_libraries(${PROJECT_NAME}Test1 ${PROJECT_NAME})

    add_executable(
        ${PROJECT_NAME}Test2
        tests/test_2.cpp
    )
    target_link_libraries(${PROJECT_NAME}Test2 ${PROJECT_NAME})

    add_executable(
        ${PROJECT_NAME}Test3
        tests/test_3.cpp
    )
    target_link_libraries(${PROJECT_NAME}Test3 ${PROJECT_NAME})

    add_executable(
        ${PROJECT_NAME}Test4
        tests/test_4.cpp
    )
    target_link_libraries(${PROJECT_NAME}Test4 ${PROJECT_NAME})
endif()