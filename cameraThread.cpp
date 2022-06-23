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
    const double focal_length_px = 100;

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

    Size imageSize = Size(848, 480);
    int imageWidth = imageSize.width;
    int imageHeight = imageSize.height;

    struct {
        Mutex mutex{};
        CameraOutputCallback callback{nullptr};
    } _camera_output_subscription{};

    static void RunThread(CameraThread *cameraThread) {
        cameraThread->run();
    }

    std::vector<Point2d> getGPSPositionOfCirclesInPicture(Mat image) {
        Mat gray;
        cvtColor(image, gray, COLOR_BGR2GRAY);
        medianBlur(gray, gray, 5);

        std::vector<Vec3f> circles;

//            todo: function (drone height) => circle radius (min, max)

        HoughCircles(gray, circles, HOUGH_GRADIENT, 1, MinDistanceBetweeenCirclesCenters_px, 100, 30, 30, 50);

        for (auto circle: circles) {
            int rowRangeStart = std::max(int(circle[1] - circle[2]), 0);
            auto rowRangeEnd = std::min(int(circle[1] + circle[2] + 1), image.rows);
            auto colRangeStart = std::max(int(circle[0] - circle[2]), 0);
            auto colRangeEnd = std::min(int(circle[0] + circle[2] + 1), image.cols);

            cv::Mat roi = image(cv::Range(rowRangeStart, rowRangeEnd),
                                cv::Range(colRangeStart, colRangeEnd));

            cv::Mat mask(roi.size(), CV_8U);
            cv::circle(mask, Point(roi.rows / 2, roi.cols / 2), int(circle[2]), cv::Scalar::all(255), -1);
            cv::Scalar roi_mean = cv::mean(roi, mask);

            cv::circle(image, Point(int(circle[0]), int(circle[1])), int(circle[2]), roi_mean, -1);

//                todo: test if color of cricle is near color that need to be detect
        }

        // pass frame to callback
        _camera_output.frame = image;

        // calculate GPS positions of detected circles
        return circlesToGPSPositions(circles);
    }

    std::vector<Point2d> filterAlreadyShootedCircles(const std::vector<Point2d> &detectedCircles) {
        std::vector<Point2d> circlesToShootGPS;

        // filter out already shooted circles
        for (const auto &new_circle: detectedCircles) {
            bool isSameCircle = false;

            for (const auto &shooted_circle: circlesAlreadyShooted) {
                auto distance_m = distanceBetweenGPSPositions_m(new_circle, shooted_circle);
//                std::cout << distance_m << " m" << std::endl;

                if (distance_m <= MaximumDistanceToBeSame_m) {
                    isSameCircle = true;
                }
            }

            if (!isSameCircle) {
                circlesToShootGPS.push_back(new_circle);
            }
        }
        return circlesToShootGPS;
    }

    void run() {
        cap.open(0);

        while (true) {
            if (!cap.isOpened()) {
                return;
            }

            Mat new_frame;
            cap >> new_frame;

            if (!new_frame.empty()) {
                std::lock_guard<Mutex> lock(_camera_output_subscription.mutex);
                std::cout << "new_frame.size: " << new_frame.size() << std::endl;
                imageSize = new_frame.size();
                break;
            }
        }

        while (keepRunning) {
            std::lock_guard<Mutex> lock(_camera_output_subscription.mutex);

            if (!cap.isOpened()) {
                std::cerr << "Camera not opened\n";

                cap.open(0);

                return;
            }

            if (_camera_output_subscription.callback == nullptr) {
                return;
            }

            Mat new_frame;
            cap >> new_frame;

            if (new_frame.empty()) {
                return;
            }

            auto detectedCircles_GPS = getGPSPositionOfCirclesInPicture(new_frame);
            std::vector<Point2d> circlesToShoot_GPS = filterAlreadyShootedCircles(detectedCircles_GPS);

            // add new circles to shoot to array already shooted
            for (const auto &c: circlesToShoot_GPS) {
                circlesAlreadyShooted.push_back(c);
            }

            _camera_output.circlesToShoot = circlesToShoot_GPS;

            _camera_output_subscription.callback(_camera_output);

            imshow("camera", new_frame);
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

    //distance in meters from image center
    double calculateDistanceFromImageCenterToCircle_m(Point2d circleCenter, double altitude_m) {
        Point2d imageCenter(imageWidth / 2, imageHeight / 2);
        Point2d circleVector = circleCenter - imageCenter;

        double cv1 = circleVector.x;
        double cv2 = circleVector.y;

        auto circleVectorLength_px = sqrt(cv1 * cv1 + cv2 * cv2);

        return (altitude_m * circleVectorLength_px) / focal_length_px;
    }

    // offset_rad angle from image
    double calculateBearingFromDroneHeadingToCirclePosition_rad(Point2d circleCenter) {
        Point2d imageCenter(imageWidth / 2, imageHeight / 2);

        Point2d baseVector = Point2d(0, -100);

        Point2d circleVector = circleCenter - imageCenter;

        auto w1 = circleVector.x;
        auto w2 = circleVector.y;
        auto w_d = sqrt(w1 * w1 + w2 * w2);

        auto v1 = baseVector.x;
        auto v2 = baseVector.y;
        auto v_d = sqrt(v1 * v1 + v2 * v2);

        auto angle_rad = acos((w1 * v1 + w2 * v2) / (w_d * v_d));

        // should return value from -pi to +pi

        if (circleCenter.x > imageCenter.x) {
            return angle_rad;
        }

        return -angle_rad;
    }

    Point2d calculateGPSPosition(Point2d circleCenter) {
        auto position = telemetry->position();

        auto altitude_m = position.relative_altitude_m;

        auto lat = position.latitude_deg;
        auto lon = position.longitude_deg;

        auto lat_rad = lat * CV_PI / 180;
        auto lon_rad = lon * CV_PI / 180;
        auto heading_rad = telemetry->heading().heading_deg * CV_PI / 180;

//    todo: CALCULATE CIRCLE CENTER OFFSET
        auto distance_m = calculateDistanceFromImageCenterToCircle_m(circleCenter, altitude_m);
        auto heading_offset_rad = calculateBearingFromDroneHeadingToCirclePosition_rad(circleCenter);

        auto brng = heading_rad + heading_offset_rad;
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