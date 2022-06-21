//
// Created by kacper on 21.06.22.
//
#include <functional>
#include <utility>
#include <mutex>

#include <opencv2/opencv.hpp>

using namespace cv;

class CameraThread {
public:

    bool keepRunning = true;

    VideoCapture cap;

    struct CameraOutput {
        Mat frame;
    };

    CameraOutput _camera_output = {};

    using CameraOutputCallback = std::function<void(CameraOutput)>;

    struct {
        Mutex mutex{};
        CameraOutputCallback callback{nullptr};
    } _camera_output_subscription{};

    CameraThread() {}

    static void RunThread(CameraThread *cameraThread) {
        cameraThread->run();
    }

    void run() {
        cap.open(0);
        while (keepRunning) {
            std::lock_guard<Mutex> lock(_camera_output_subscription.mutex);

            if (!cap.isOpened()) {
                std::cerr << "Camera not opened\n";
                return;
            }

            if (_camera_output_subscription.callback == nullptr) {
                return;
            }

            Mat tmp_frame;
            cap >> tmp_frame;

            if (tmp_frame.empty()) {
                return;
            }

            Mat gray;
            cvtColor(tmp_frame, gray, COLOR_BGR2GRAY);
            medianBlur(gray, gray, 5);
            std::vector<Vec3f> circles;
            HoughCircles(gray, circles, HOUGH_GRADIENT, 1, gray.rows / 16, 100, 30, 1, 30);
            for (auto &i: circles) {
                Vec3i c = i;
                Point center = Point(c[0], c[1]);

                // circle center
                circle(tmp_frame, center, 1, Scalar(0, 100, 100), 3, LINE_AA);

                // circle outline
                int radius = c[2];
                circle(tmp_frame, center, radius, Scalar(255, 0, 255), 3, LINE_AA);
            }

            _camera_output.frame = tmp_frame;
            _camera_output_subscription.callback(_camera_output);
        }
    }

    void subscribe_camera_output(CameraOutputCallback callback) {
        std::lock_guard<Mutex> lock(_camera_output_subscription.mutex);
        _camera_output_subscription.callback = std::move(callback);
    }

    void stop() {
        std::lock_guard<Mutex> lock(_camera_output_subscription.mutex);
        keepRunning = false;
    }
};