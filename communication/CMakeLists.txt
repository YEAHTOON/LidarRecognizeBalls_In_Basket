cmake_minimum_required(VERSION 3.0)

project(com)

#设置变量
set(CODE_DIR "./code")                  #代码的文件夹
set(LIB_OUT "../lib")    #静态库的存放文件夹

#设置源文件文件夹
aux_source_directory(./code/uart UART)  
aux_source_directory(./code/tcp TCP)  
aux_source_directory(./code/thread THREAD)  

#包含头文件
include_directories(${CODE_DIR}/uart ${CODE_DIR}/tcp ${CODE_DIR}/thread ${CODE_DIR}/log)

#生成静态库
add_library(yztuart STATIC ${UART})
add_library(yzttcp STATIC ${TCP})
add_library(yztthread STATIC ${THREAD})
set_target_properties(yztuart PROPERTIES ARCHIVE_OUTPUT_DIRECTORY ${LIB_OUT})  #输出目录
set_target_properties(yzttcp PROPERTIES ARCHIVE_OUTPUT_DIRECTORY ${LIB_OUT})
set_target_properties(yztthread PROPERTIES ARCHIVE_OUTPUT_DIRECTORY ${LIB_OUT})

#生成可执行文件
add_executable(com ${CODE_DIR}/main/main.cpp)
target_link_libraries(com yztuart)
target_link_libraries(com yzttcp)
target_link_libraries(com yztthread)
