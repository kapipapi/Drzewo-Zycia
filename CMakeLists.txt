cmake_minimum_required(VERSION 3.10.2)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

project(drzewo_zycia)

add_subdirectory(camera_calibration)
add_subdirectory(algorithm_wip)
add_subdirectory(photo_test)
add_subdirectory(live_video_test)
add_subdirectory(gsteam_test)

add_executable(drzewo_zycia main.cpp cameraThread.cpp)

find_package(MAVSDK REQUIRED)

target_link_libraries(drzewo_zycia
        MAVSDK::mavsdk
        )

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_compile_options(drzewo_zycia PRIVATE -Wall -Wextra)

target_link_libraries(drzewo_zycia ${OpenCV_LIBS})