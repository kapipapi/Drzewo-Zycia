//
// Created by kacper on 25.06.22.
//

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>

#include "../cameraThread.cpp"


int main() {
    CameraThread camera{};

    camera.pixel_calculation_error_rate = 0.2;

    camera.cap.open(2);
    if (!camera.cap.isOpened()) {
        std::cout << "errrr" << std::endl;
    }

    cv::namedWindow("original", 0);
    cv::namedWindow("video", 0);

    double altitude_m = 10;

    while (true) {
        cv::Mat frame;

        camera.cap >> frame;

        if (frame.empty()) {
            continue;
        }

        cv::Mat cam_intrinsic(3, 3, CV_64FC1);
        cam_intrinsic.at<double>(0, 0) = 986.0116328851432;
        cam_intrinsic.at<double>(1, 0) = 0.0;
        cam_intrinsic.at<double>(2, 0) = 0.0;

        cam_intrinsic.at<double>(0, 1) = 0.0;
        cam_intrinsic.at<double>(1, 1) = 982.0788151145993;
        cam_intrinsic.at<double>(2, 1) = 0.0;

        cam_intrinsic.at<double>(0, 2) = 687.6426355899254;
        cam_intrinsic.at<double>(1, 2) = 365.6470526046041;
        cam_intrinsic.at<double>(2, 2) = 1.0;

        cv::Mat distCoeffs(5, 1, CV_64FC1);
        distCoeffs.at<double>(0) = -0.4975842106721171;
        distCoeffs.at<double>(1) = 0.2559664966234141;
        distCoeffs.at<double>(2) = 0.003222902493099889;
        distCoeffs.at<double>(3) = -0.0001964051137438205;
        distCoeffs.at<double>(4) = -0.05437674161052661;

        Mat imageUndistorted;
        undistort(frame, imageUndistorted, cam_intrinsic, distCoeffs);
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
        drawContours(frame, squares, -1, Scalar::all(255));

        cv::imshow("original", frame);
        cv::imshow("video", imageUndistorted);

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