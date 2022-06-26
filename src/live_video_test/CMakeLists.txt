add_executable(live_video_test main.cpp ../../cameraThread.cpp)

target_link_libraries(live_video_test ${OpenCV_LIBS} MAVSDK::mavsdk camera_thread)