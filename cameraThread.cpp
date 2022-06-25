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
    const double focal_length_px = 2000;

    const double circleRadius_m = 0.5;
    const double squareSide_m = 0.5;
    double pixel_calculation_error_rate = 0.2; // 10% up and down

    std::string videoPath;

public:

    CameraThread() = default;

    CameraThread(Telemetry *telemetry, const std::string &videoPath = "") {
        this->telemetry = telemetry;
        this->videoPath = videoPath;
    }

    bool DRAWING = false;

    Telemetry *telemetry;

    bool keepRunning = true;

    enum TreeType {
        healthy_tree,
        vulnerable_tree,
        pathogen_gold_tree,
        pathogen_beige_tree,
        probably_grass
    };

    Scalar healthy{255, 255, 255};
    Scalar vulnerable{147, 107, 76};
    Scalar pathogen_gold{212, 159, 65};
    Scalar pathogen_beige{249, 246, 227};

    std::vector<std::pair<Scalar, TreeType>> circle_colors{
            std::pair{vulnerable, vulnerable_tree},
            std::pair{pathogen_gold, pathogen_gold_tree},
            std::pair{pathogen_beige, pathogen_beige_tree}
    };

    struct Circle {
        double x;
        double y;
        double radius;
        TreeType type;
    };

    struct Tree {
        double lat;
        double lon;
        TreeType type;
    };

    VideoCapture cap;

    struct CameraOutput {
        Mat frame;
        std::vector<Tree> circlesToShoot;
    };

    std::vector<Tree> circlesAlreadyShooted;

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

    Scalar getColorFromType(TreeType type) {
        switch (type) {
            case healthy_tree:
                return healthy;
            case vulnerable_tree:
                return vulnerable;
            case pathogen_gold_tree:
                return pathogen_gold;
            case pathogen_beige_tree:
                return pathogen_beige;
            case probably_grass :
                return {0, 255, 0};
            default:
                return {255, 255, 255};
        }
    }

    String getStringFromType(TreeType type) {
        switch (type) {
            case healthy_tree:
                return "healthy";
            case vulnerable_tree:
                return "vulnerable";
            case pathogen_gold_tree:
                return "pathogen_gold";
            case pathogen_beige_tree:
                return "pathogen_beige";
            case probably_grass :
                return "probably_grass";
            default:
                return "notype";
        }
    }

    double calculateColorDifference(Scalar a, Scalar z) {
        double dr = a[0] - z[0],
                dg = a[1] - z[1],
                db = a[2] - z[2];

        return sqrt(dr * dr + dg * dg + db * db);
    }


    TreeType getTreeType(Scalar tree) {
        std::pair<double, TreeType> min{50, probably_grass};
        for (const auto &c: circle_colors) {
            auto diff = calculateColorDifference(tree, c.first);
            if (diff < min.first) {
                min = std::pair{diff, c.second};
            }
        }

        return min.second;
    }

    std::pair<int, int> getCircleRadius(double altitude_m) {
        double approx_radius_px = (circleRadius_m * focal_length_px) / altitude_m;

        int min_rad = max(int(approx_radius_px * (1 - pixel_calculation_error_rate)), 0);
        int max_rad = max(int(approx_radius_px * (1 + pixel_calculation_error_rate)), 0);

        return std::pair{min_rad, max_rad};
    }

    std::vector<Circle> getCirclesInImage(
            Mat image,
            double altitude_m) {
        Mat gray;
        cvtColor(image, gray, COLOR_BGR2GRAY);
        medianBlur(gray, gray, 5);

        std::vector<Vec3f> circles;

        std::pair circle_radius = getCircleRadius(altitude_m);

        HoughCircles(gray, circles, HOUGH_GRADIENT, 1, MinDistanceBetweeenCirclesCenters_px, 100, 30,
                     circle_radius.first, circle_radius.second);


        std::vector<Circle> output_circles;
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

            if (DRAWING) cv::circle(image, Point(int(circle[0]), int(circle[1])), int(circle[2]), roi_mean, -1);

            Circle circ{circle[0], circle[1], circle[2], getTreeType(roi_mean)};

            if (circ.type != probably_grass) {
                if (circ.type == vulnerable_tree) {
                    std::cout << roi_mean << std::endl;
                }
                output_circles.push_back(circ);
            }
        }

        // pass frame to callback
        _camera_output.frame = image;

        // calculate GPS positions of detected circles
        return output_circles;
    }

    std::vector<Tree> filterAlreadyShootedCircles(const std::vector<Tree> &detectedCircles) {
        std::vector<Tree> circlesToShootGPS;

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

    static double angle(Point pt1, Point pt2, Point pt0) {
        double dx1 = pt1.x - pt0.x;
        double dy1 = pt1.y - pt0.y;
        double dx2 = pt2.x - pt0.x;
        double dy2 = pt2.y - pt0.y;
        return (dx1 * dx2 + dy1 * dy2) / sqrt((dx1 * dx1 + dy1 * dy1) * (dx2 * dx2 + dy2 * dy2) + 1e-10);
    }

    bool isSquareAreaOK(double area_pxpx, double altitude_m) {
        double approx_side_px = (squareSide_m * focal_length_px) / altitude_m;

        double apporx_area_pxpx = approx_side_px * approx_side_px;

        double min_area_pxpx = apporx_area_pxpx * (1 - pixel_calculation_error_rate);
        double max_area_pxpx = apporx_area_pxpx * (1 + pixel_calculation_error_rate);

        return min_area_pxpx < area_pxpx && area_pxpx < max_area_pxpx;
    }

    std::vector<std::vector<Point>>
    findHealthyTrees(const Mat &image, double altitude_m) {
        Mat gray0(image.size(), CV_8U), gray;
        int thresh = 50, N = 2;
//        cvtColor(image, gray, COLOR_BGR2GRAY);
//        medianBlur(gray, gray, 5);

        std::vector<std::vector<Point>> contours;
        std::vector<std::vector<Point>> squares;

        for (int l = 0; l < N; l++) {
            // hack: use Canny instead of zero threshold level.
            // Canny helps to catch squares with gradient shading
            if (l == 0) {
                // apply Canny. Take the upper threshold from slider
                // and set the lower to 0 (which forces edges merging)
                Canny(gray0, gray, 0, thresh, 5);
                // dilate canny output to remove potential
                // holes between edge segments
                dilate(gray, gray, Mat(), Point(-1, -1));
            } else {
                // apply threshold if l!=0:
                //     tgray(x,y) = gray(x,y) < (l+1)*255/N ? 255 : 0
                gray = gray0 >= (l + 1) * 255 / N;
            }
            // find contours and store them all as a list
            findContours(gray, contours, RETR_LIST, CHAIN_APPROX_SIMPLE);
            std::vector<Point> approx;
            // test each contour
            for (size_t i = 0; i < contours.size(); i++) {
                // approximate contour with accuracy proportional
                // to the contour perimeter
                approxPolyDP(contours[i], approx, arcLength(contours[i], true) * 0.02, true);
                // square contours should have 4 vertices after approximation
                // relatively large area (to filter out noisy contours)
                // and be convex.
                // Note: absolute value of an area is used because
                // area may be positive or negative - in accordance with the
                // contour orientation
                auto area = fabs(contourArea(approx));
                if (approx.size() == 4 &&
                    isSquareAreaOK(area, altitude_m) &&
                    isContourConvex(approx)) {

                    double maxCosine = 0;
                    for (int j = 2; j < 5; j++) {
                        // find the maximum cosine of the angle between joint edges
                        double cosine = fabs(angle(approx[j % 4], approx[j - 2], approx[j - 1]));
                        maxCosine = MAX(maxCosine, cosine);
                    }
                    // if cosines of all angles are small
                    // (all angles are ~90 degree) then write quandrange
                    // vertices to resultant sequence
                    if (maxCosine < 0.3) {
                        auto rect = boundingRect(approx);
                        Mat mask(rect.height, rect.width, CV_8U);
                        drawContours(mask, std::vector{approx}, 0, Scalar::all(255), -1);
                        cv::Scalar rect_mean = cv::mean(image(rect), mask);
                        auto color_diff = calculateColorDifference(healthy, rect_mean);

                        if (color_diff > 50) {
                            continue;
                        }

                        if (DRAWING) drawContours(image, std::vector{approx}, 0, Scalar::all(0), 3);

                        squares.push_back(approx);
                    }
                }
            }
        }

        return squares;
    }

    void run() {
        if (videoPath.empty()) {
            cap.open(0);
        } else {
            cap.open(videoPath);
        }

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

            auto position = telemetry->position();
            auto heading_deg = telemetry->heading().heading_deg;

            if (!cap.isOpened()) {
                std::cerr << "Camera not opened\n";

                if (videoPath.empty()) {
                    cap.open(0);
                } else {
                    cap.open(videoPath);
                }

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

            auto squares = findHealthyTrees(new_frame, position.relative_altitude_m);
            std::vector<Tree> healthy_trees;
            for (auto sq: squares) {
                auto M = moments(sq);
                double cX = M.m10 / M.m00;
                double cY = M.m01 / M.m00;
                healthy_trees.push_back(calculateGPSPosition(Point(cX, cY), healthy_tree, position, heading_deg));
            }

            auto detectedCircles = getCirclesInImage(new_frame, position.relative_altitude_m);
            auto detectedCircles_GPS = circlesToGPSPositions(detectedCircles, position, heading_deg);
            std::vector<Tree> circlesToShoot_GPS = filterAlreadyShootedCircles(detectedCircles_GPS);

            // add new circles to shoot to array already shot
            for (const auto &c: circlesToShoot_GPS) {
                circlesAlreadyShooted.push_back(c);
            }

            _camera_output.circlesToShoot = circlesToShoot_GPS;

            _camera_output_subscription.callback(_camera_output);

            imshow("camera", new_frame);
            waitKey(10);
        }
    }

    std::vector<Tree> circlesToGPSPositions(const std::vector<Circle> &circles,
                                            mavsdk::Telemetry::Position position,
                                            double heading_deg) {
        std::vector<Tree> tmpCirclesPositions;
        for (auto c: circles) {
            auto center = Point2d(c.x, c.y);
            tmpCirclesPositions.push_back(calculateGPSPosition(center, c.type, position, heading_deg));
        }

        return tmpCirclesPositions;
    }

    //distance in meters from image center
    double calculateDistanceFromImageCenterToCircle_m(const Point2d &circleCenter, double altitude_m) {
        Point2d imageCenter(float(imageWidth) / 2, float(imageHeight) / 2);
        Point2d circleVector = circleCenter - imageCenter;

        double cv1 = circleVector.x;
        double cv2 = circleVector.y;

        auto circleVectorLength_px = sqrt(cv1 * cv1 + cv2 * cv2);

        return (altitude_m * circleVectorLength_px) / focal_length_px;
    }

    // offset_rad angle from image
    double calculateBearingFromDroneHeadingToCirclePosition_rad(const Point2d &circleCenter) {
        Point2d imageCenter(float(imageWidth) / 2, float(imageHeight) / 2);

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

    Tree calculateGPSPosition(const Point2d &circleCenter, TreeType type, mavsdk::Telemetry::Position position,
                              double heading_deg) {
        auto altitude_m = position.relative_altitude_m;

        auto lat = position.latitude_deg;
        auto lon = position.longitude_deg;

        auto lat_rad = lat * CV_PI / 180;
        auto lon_rad = lon * CV_PI / 180;
        auto heading_rad = heading_deg * CV_PI / 180;

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

        return {target_lat, target_lon, type};
    }

    double distanceBetweenGPSPositions_m(const Tree &p1, const Tree &p2) {
        auto lat1 = p1.lat;
        auto lon1 = p1.lon;

        auto lat2 = p2.lat;
        auto lon2 = p2.lon;

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