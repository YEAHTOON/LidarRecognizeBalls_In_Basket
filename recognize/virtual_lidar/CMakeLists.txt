cmake_minimum_required(VERSION 3.0)

project(vir)

include_directories(../code/tcp)

add_executable(vir main.cpp ../code/tcp/tcp.cpp)
