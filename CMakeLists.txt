cmake_minimum_required(VERSION 3.10.2)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

project(drzewo_zycia)

include(cmake/build_external_project.cmake)

#build_external_project(
#        mavsdk
#        https://github.com/mavlink/MAVSDK.git
#        tags/v1.4.2
#        -DCMAKE_INSTALL_PREFIX:PATH=${CMAKE_CURRENT_BINARY_DIR}/mavsdk
#)
set(MAVSDK_DIR ${CMAKE_CURRENT_BINARY_DIR}/mavsdk/lib/cmake/MAVSDK)

find_package(MAVSDK REQUIRED)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_subdirectory(src)

add_executable(drzewo_zycia main.cpp cameraThread.cpp)

target_link_libraries(
        drzewo_zycia
        MAVSDK::mavsdk
        ${OpenCV_LIBS}
)
