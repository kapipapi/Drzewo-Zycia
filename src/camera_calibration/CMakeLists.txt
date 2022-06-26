add_executable(camera_calibration main.cpp)

target_link_libraries(camera_calibration ${OpenCV_LIBS} MAVSDK:mavsdk)