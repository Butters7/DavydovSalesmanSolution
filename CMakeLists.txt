cmake_minimum_required(VERSION 3.0.0)
project(HW5)

add_executable(HW5 main.cpp)

add_subdirectory(lib)

target_link_libraries(${PROJECT_NAME} PUBLIC dsu fcgraph)
