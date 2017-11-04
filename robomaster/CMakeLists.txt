

CMAKE_MINIMUM_REQUIRED(VERSION 3.5.1)
PROJECT(masters CXX)

# 隐含定义了两个变量
# masters_BINARY_DIR 和 masters_SOURCE_DIR 内部工程，这两个变量都指向
#当前文件夹，同时也自动定义了 两个变量
# PROJECT_BINARY_DIR 和 PROJECT_SOURCE_DIR  这两个变量和上面两个变量一样

#SET 指令
#用来显示定义变量 
# 支持C++11标准
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)
SET(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)
SET(CMAKE_PREFIX_PATH /usr/local/OpenCV/share)
FIND_PACKAGE(OpenCV REQUIRED)

MESSAGE(STATUS "OpenCV library status:")
MESSAGE(STATUS " version : ${opencv_VERSION}")
MESSAGE(STATUS " include path: ${OpenCV_INCLUDE_DIRS}")
MESSAGE(STATUS " library: ${OpenCV_LIBS}")

# ADD opencv headers include paths
INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})

#add opencv libs
AUX_SOURCE_DIRECTORY(. DIR_SRCS)

MESSAGE(STATUS " DIR_SRCS  ${DIR_SRCS}")

# add executable file of masters
ADD_EXECUTABLE(masters ${DIR_SRCS})

TARGET_LINK_LIBRARIES(masters ${OpenCV_LIBS};pthread)

