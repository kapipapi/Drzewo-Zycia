//
// Created by kacper on 25.06.22.
//

//
// Created by kacper on 25.06.22.
//

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>

#include "../cameraThread.cpp"


int main() {
    Mat frame = imread("/home/kacper/Pictures/DJI_202206260050_013/DJI_20220626005511_0006.JPG");

    CameraThread camera{};

    camera.pixel_calculation_error_rate = 0.3;

    camera.focal_length_px = 16867.768;

    double altitude_m = 32.068;

    auto trees = camera.findHealthyTrees(frame, altitude_m);
    drawContours(frame, trees, -1, Scalar(0, 0, 0), 10);
    std::cout << trees.size() << std::endl;

    resize(frame, frame, Size(8192 / 4, 5460 / 4));
    cv::namedWindow("frame", CV_WINDOW_FULLSCREEN);
    imshow("frame", frame);

    waitKey();

    return 0;
}