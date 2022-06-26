add_library(camera_thread cameraThread.cpp)

target_link_libraries(camera_thread ${OpenCV_LIBS} MAVSDK::mavsdk)
