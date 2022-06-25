cmake_minimum_required(VERSION 3.10.2)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

project(live_video_test)

add_executable(live_video_test main.cpp ../cameraThread.cpp)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_compile_options(live_video_test PRIVATE -Wall -Wextra)

target_link_libraries(live_video_test ${OpenCV_LIBS})