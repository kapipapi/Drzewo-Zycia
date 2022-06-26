add_executable(photo_test main.cpp)

target_link_libraries(photo_test ${OpenCV_LIBS} MAVSDK:mavsdk)