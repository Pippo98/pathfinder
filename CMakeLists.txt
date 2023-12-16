cmake_minimum_required(VERSION 3.15)
project(RRTStar)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_CXX_FLAGS "-O3 -g")

include_directories(
    ${CMAKE_CURRENT_LIST_DIR}
)

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
    AStarTest1
    tests/test_4.cpp
)
target_link_libraries(AStarTest1 ${PROJECT_NAME})