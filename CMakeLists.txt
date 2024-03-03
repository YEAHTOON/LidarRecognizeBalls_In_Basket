cmake_minimum_required(VERSION 3.0)

# SET(CMAKE_BUILD_TYPE "Debug")
# SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")
# SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")

project(mid)

find_package(PCL REQUIRED)

add_executable(mid main.cpp)

target_link_libraries(mid livox_lidar_sdk_static)
target_link_libraries(mid ${PCL_LIBRARIES})