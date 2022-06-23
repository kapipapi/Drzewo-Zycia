//
// Created by kacper on 21.06.22.
//
#include <functional>
#include <utility>
#include <mutex>
#include <future>

#include <opencv2/opencv.hpp>
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/telemetry/telemetry.h>

using namespace cv;
using namespace mavsdk;

class CameraThread {
    const double MaximumDistanceToBeSame_m = 1; //center to center in meters
    const double MinDistanceBetweeenCirclesCenters_px = 500;

public:

    CameraThread(Telemetry *telemetry) {
        this->telemetry = telemetry;
    }

    Telemetry *telemetry;

    bool keepRunning = true;

    VideoCapture cap;

    struct CameraOutput {
        Mat frame;
        std::vector<Point2d> circlesToShoot;
    };

    std::vector<Point2d> circlesAlreadyShooted;

    CameraOutput _camera_output = {};

    using CameraOutputCallback = std::function<void(CameraOutput)>;

    struct {
        Mutex mutex{};
        CameraOutputCallback callback{nullptr};
    } _camera_output_subscription{};

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

//            todo: function (drone height) => circle radius (min, max)

            HoughCircles(gray, circles, HOUGH_GRADIENT, 1, MinDistanceBetweeenCirclesCenters_px, 100, 30, 30, 50);
            for (auto &i: circles) {
                Vec3i c = i;
                Point center = Point(c[0], c[1]);

                // circle center
                circle(tmp_frame, center, 1, Scalar(0, 100, 100), 3, LINE_AA);
                putText(tmp_frame, std::to_string(c[2]), center, 1, 2, Scalar(255, 0, 255));

                // circle outline
                int radius = c[2];
                circle(tmp_frame, center, radius, Scalar(255, 0, 255), 3, LINE_AA);
            }

            for (auto circle: circles) {
                int rowRangeStart = std::max(int(circle[1] - circle[2]), 0);
                auto rowRangeEnd = std::min(int(circle[1] + circle[2] + 1), tmp_frame.rows);
                auto colRangeStart = std::max(int(circle[0] - circle[2]), 0);
                auto colRangeEnd = std::min(int(circle[0] + circle[2] + 1), tmp_frame.cols);

                cv::Mat roi = tmp_frame(cv::Range(rowRangeStart, rowRangeEnd),
                                        cv::Range(colRangeStart, colRangeEnd));

                cv::Mat1i mask(roi.size());
                cv::circle(mask, Point(roi.rows / 2, roi.cols / 2), circle[2], cv::Scalar::all(255), -1);
                cv::Scalar roi_mean = cv::mean(roi, mask);

//                todo: test if color of cricle is near color that need to be detect
            }

            // pass frame to callback
            _camera_output.frame = tmp_frame;

            // calculate GPS positions of detected circles
            auto detectedCirclesGPS = circlesToGPSPositions(circles);
            std::vector<Point2d> circlesToShootGPS;

            // filter out already shooted circles
            for (const auto &new_circle: detectedCirclesGPS) {
                bool isSameCircle = false;

                for (const auto &shooted_circle: circlesAlreadyShooted) {
                    auto distance_m = distanceBetweenGPSPositions_m(new_circle, shooted_circle);
                    std::cout << distance_m << " m" << std::endl;

                    if (distance_m <= MaximumDistanceToBeSame_m) {
                        isSameCircle = true;
                    }
                }

                if (!isSameCircle) {
                    circlesToShootGPS.push_back(new_circle);
                }
            }

            // add new circles to shoot to array already shooted
            for (const auto &c: circlesToShootGPS) {
                circlesAlreadyShooted.push_back(c);
            }

            _camera_output.circlesToShoot = circlesToShootGPS;

            _camera_output_subscription.callback(_camera_output);

            imshow("camera", tmp_frame);
            waitKey(10);
        }
    }

    std::vector<Point2d> circlesToGPSPositions(const std::vector<Vec3f> &circles) {
        std::vector<Point2d> tmpCirclesPositions;
        for (auto c: circles) {
            auto center = Point2d(c[0], c[1]);
            tmpCirclesPositions.push_back(calculateGPSPosition(center));
        }
        return tmpCirclesPositions;
    }

//    todo: CALCULATE CIRCLE CENTER OFFSET
    Point2d calculateGPSPosition(Point2d imageCenter) {
        auto position = telemetry->position();

        auto lat = position.latitude_deg;
        auto lon = position.longitude_deg;

        auto lat_rad = lat * CV_PI / 180;
        auto lon_rad = lon * CV_PI / 180;
        auto heading = telemetry->heading().heading_deg * CV_PI / 180;

        auto distance_m = 0; //distance in meters from image center
        auto offset = 0; // offset angle from image

        auto brng = heading - offset;
        const auto R_m = 6336000; // earth radius in meters

        auto target_lat_rad = asin(
                sin(lat_rad) * cos(distance_m / R_m) + cos(lat_rad) * sin(distance_m / R_m) * cos(brng));
        auto target_lon_rad = lon_rad + atan2(sin(brng) * sin(distance_m / R_m) * cos(lat_rad),
                                              cos(distance_m / R_m) - sin(lat_rad) * sin(target_lat_rad));

        auto target_lat = target_lat_rad * 180 / CV_PI;
        auto target_lon = target_lon_rad * 180 / CV_PI;

        return {target_lat, target_lon};
    }

    double distanceBetweenGPSPositions_m(const Point2d &p1, const Point2d &p2) {
        auto lat1 = p1.x;
        auto lon1 = p1.y;

        auto lat2 = p2.x;
        auto lon2 = p2.y;

        const auto R_m = 6336000; // metres
        const auto lat1_rad = lat1 * CV_PI / 180;
        const auto lat2_rad = lat2 * CV_PI / 180;
        const auto lat_diff = (lat2 - lat1) * CV_PI / 180;
        const auto lon_diff = (lon2 - lon1) * CV_PI / 180;

        const auto a = sin(lat_diff / 2) * sin(lat_diff / 2) +
                       cos(lat1_rad) * cos(lat2_rad) *
                       sin(lon_diff / 2) * sin(lon_diff / 2);
        const auto c = 2 * atan2(sqrt(a), sqrt(1 - a));

        return R_m * c; // in metres
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