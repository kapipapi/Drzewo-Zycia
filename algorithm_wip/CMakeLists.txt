add_executable(algorithm_wip main.cpp ../cameraThread.cpp)

target_link_libraries(algorithm_wip ${OpenCV_LIBS} MAVSDK::mavsdk)
