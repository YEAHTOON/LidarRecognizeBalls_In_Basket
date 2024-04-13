cmake_minimum_required(VERSION 3.0)

# SET(CMAKE_BUILD_TYPE "Debug")
# SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")
# SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")

project(mid)

find_package(Python COMPONENTS Interpreter Development REQUIRED)
include_directories(${Python_INCLUDE_DIRS})

find_package(PCL REQUIRED)
include_directories(./code/log)

add_executable(mid main.cpp ./code/tcp/tcp.cpp)

target_link_libraries(mid livox_lidar_sdk_static)
target_link_libraries(mid ${PCL_LIBRARIES})
target_link_libraries(mid ${Python_LIBRARIES})