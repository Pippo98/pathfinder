cmake_minimum_required(VERSION 3.15)
project(RRTStar)

set(CMAKE_CXX_STANDARD 17)

include_directories(${CMAKE_CURRENT_LIST_DIR})

add_library(
    ${PROJECT_NAME}
    src/rrt_star.cpp

    src/point.cpp
    src/polygon.cpp
)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/bin)

add_executable(
    ${PROJECT_NAME}Test1
    tests/test_1.cpp
)
target_link_libraries(${PROJECT_NAME}Test1 ${PROJECT_NAME})