//
// Created by kacper on 26.06.22.
//

#include <opencv2/opencv.hpp>

int main() {

    std::string stream_string = "appsrc ! video/x-raw, format=BGR ! queue ! videoconvert ! video/x-raw, format=BGRx ! nvvidconv ! omxh264enc ! video/x-h264, stream-format=byte-stream ! h264parse ! rtph264pay pt=96 config-interval=1 ! udpsink host=localhost port=50001";
    cv::VideoWriter stream(stream_string, 0, 30, cv::Size(1280, 720));

    cv::VideoCapture cap;
    cap.open(0);

    if (!cap.isOpened() || !stream.isOpened()) {
        std::cout << "stream or capture isnt open" << std::endl;
        return 1;
    }

    cv::Mat frame;
    while (true) {
        cap >> frame;
        stream.write(frame);
    }

    return 0;
}