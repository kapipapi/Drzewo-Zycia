//
// Created by kacper on 26.06.22.
//

#include <iostream>
#include <opencv2/opencv.hpp>
#include <chrono>
#include <thread>

int main() {

    //std::string stream_string = "appsrc ! video/x-raw, format=BGR ! queue ! videoconvert ! video/x-raw, format=BGRx ! nvvidconv ! omxh264enc preset-level=1 bitrate=5000  ! video/x-h264, stream-format=byte-stream ! h264parse ! rtph264pay pt=96 config-interval=1 ! udpsink host=localhost port=5602";
    std::string stream_string = "appsrc ! video/x-raw, format=BGR ! queue ! videoconvert ! video/x-raw, format=BGRx ! nvvidconv ! nvv4l2h264enc maxperf-enable=1 insert-sps-pps=1 ! h264parse ! rtph264pay ! udpsink host=localhost port=5602";
    //std::string stream_string = "appsrc ! video/x-raw, format=BGR ! queue ! videoconvert ! video/x-raw, format=BGRx !videoconvert ! x264enc speed-preset=ultrafast tune=zerolatency bitrate=10000 key-int-max=10 ! queue ! rtph264pay ! udpsink host=127.0.0.1 port=5602";
    cv::VideoWriter stream(stream_string, 0, 30, cv::Size(1280, 720));


    auto cap = cv::VideoCapture("v4l2src device=/dev/video0 ! jpegdec ! queue! videoconvert ! appsink", cv::CAP_GSTREAMER);

    if (!cap.isOpened() || !stream.isOpened()) {
        std::cout << "stream or capture isnt open" << std::endl;
        return 1;
    }

    cv::Mat frame;
    while (true) {
	std::chrono::milliseconds timespan(33); // or whatever
	std::this_thread::sleep_for(timespan);
        cap >> frame;
        stream.write(frame);
    }

    return 0;
}
