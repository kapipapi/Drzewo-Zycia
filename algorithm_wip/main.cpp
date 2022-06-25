//
// Created by kacper on 25.06.22.
//

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>

#include "../cameraThread.cpp"


int main() {

    CameraThread camera{};

    camera.cap.open("/home/kacper/Downloads/chudoby_01.avi");

    cv::namedWindow("video", 0);

    double altitude_m = 16;

    auto r = camera.getCircleRadius(altitude_m);

    std::cout << r.first << " - " << r.second << std::endl;

    while (true) {
        cv::Mat frame;

        camera.cap >> frame;

        auto circles = camera.getCirclesInImage(frame, altitude_m);

        for (auto c: circles) {
            if (c.type != CameraThread::probably_grass) {
                cv::circle(frame, Point(int(c.x), int(c.y)), int(c.radius), camera.getColorFromType(c.type), 5);
                cv::putText(frame, std::to_string(int(c.radius)), Point(int(c.x), int(c.y)), 1, 1, Scalar::all(0), 2);
                cv::putText(frame, camera.getStringFromType(c.type), Point(int(c.x), int(c.y + 10)), 1, 1,
                            Scalar::all(0),
                            2);
            }
        }

        auto squares = camera.findHealthyTrees(frame, altitude_m);

        cv::imshow("video", frame);

        switch (cv::waitKey(1)) {
            case 's':
                cv::imwrite("../../algorithm_wip/test.jpg", frame);
                break;
            case 'q':
                goto exit;
            default:
                continue;
        }
    }
    exit:

    return 0;
}