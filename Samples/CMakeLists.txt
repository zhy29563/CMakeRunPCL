# 指定最低的CMAKE版本
CMAKE_MINIMUM_REQUIRED(VERSION 3.22 FATAL_ERROR)

# 创建项目
PROJECT(PCLSamples LANGUAGES CXX)

# 指定CPLUSPLUS标准
SET(CMAKE_CXX_STANDARD_REQUIRED ON)
SET(CMAKE_CXX_STANDARD 14)
SET(CMAKE_C_STANDARD 14)
SET(CMAKE_C_STANDARD_REQUIRED ON)

# 编译器定义
ADD_COMPILE_DEFINITIONS(_CRT_SECURE_NO_WARNINGS)

# 查找第三方依赖库
FIND_PACKAGE(PCL 1.11 REQUIRED)
ADD_DEFINITIONS(${PCL_DEFINITIONS})
INCLUDE_DIRECTORIES(${PCL_INCLUDE_DIRS})
LINK_DIRECTORIES(${PCL_LIBRARY_DIRS})
LINK_LIBRARIES(${PCL_LIBRARIES})

########################################################################################################################
# I/O
########################################################################################################################
ADD_EXECUTABLE(io_pcd_file                  sources/io/pcd_file.cpp)
ADD_EXECUTABLE(io_pcd_write                 sources/io/pcd_write.cpp)
ADD_EXECUTABLE(io_pcd_read                  sources/io/pcd_read.cpp)