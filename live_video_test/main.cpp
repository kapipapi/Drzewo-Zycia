//
// Created by kacper on 25.06.22.
//

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>

#include "../cameraThread.cpp"

using namespace mavsdk;

using std::chrono::seconds;
using std::chrono::milliseconds;
using std::this_thread::sleep_for;

std::shared_ptr<System> get_system(Mavsdk &mavsdk) {
    std::cout << "Waiting to discover system...\n";
    auto prom = std::promise<std::shared_ptr<System>>{};
    auto fut = prom.get_future();

    // We wait for new systems to be discovered, once we find one that has an
    // autopilot, we decide to use it.
    mavsdk.subscribe_on_new_system([&mavsdk, &prom]() {
        auto system = mavsdk.systems().back();

        if (system->has_autopilot()) {
            std::cout << "Discovered autopilot\n";

            // Unsubscribe again as we only want to find one system.
            mavsdk.subscribe_on_new_system(nullptr);
            prom.set_value(system);
        }
    });

    // We usually receive heartbeats at 1Hz, therefore we should find a
    // system after around 3 seconds max, surely.
    if (fut.wait_for(seconds(3)) == std::future_status::timeout) {
        std::cerr << "No autopilot found.\n";
        return {};
    }

    // Get discovered system now.
    return fut.get();
}

int main(int argc, char *argv[]) {
    if (argc < 2) {
        std::cout
                << "./main [connection string - sim: udp://:14540]";
        return -1;
    }

    std::string connectionString = argv[1];

    Mavsdk mavsdk;
    ConnectionResult connection_result = mavsdk.add_any_connection(connectionString);

    if (connection_result != ConnectionResult::Success) {
        std::cerr << "Connection failed: " << connection_result << '\n';
        return 1;
    }

    auto system = get_system(mavsdk);
    if (!system) {
        return 1;
    }

    // Instantiate plugins.
    auto telemetry = Telemetry{system};
    CameraThread camera{&telemetry, ""};

    camera.cap.open(0);
    if (!camera.cap.isOpened()) {
        std::cout << "errrr" << std::endl;
    }

    int i = 0;
    while (true) {
        cv::Mat frame = camera.getFreshFrame();
        auto position = telemetry.position();
        auto heading_deg = telemetry.heading().heading_deg;

        if (frame.empty()) {
            continue;
        }

//        CIRCLES
        auto detectedCircles = camera.getCirclesInImage(frame, position.relative_altitude_m);
        auto detectedCircles_GPS = camera.circlesToGPSPositions(detectedCircles, position, heading_deg);
        std::vector<CameraThread::Tree> circlesToShoot_GPS = camera.filterAlreadyShootedCircles(detectedCircles_GPS);
        for (auto c: detectedCircles) {
            if (c.type != CameraThread::probably_grass) {
                cv::circle(frame, Point(int(c.x), int(c.y)), int(c.radius), camera.getColorFromType(c.type), 5);
                cv::putText(frame, std::to_string(int(c.radius)), Point(int(c.x), int(c.y)), 1, 1, Scalar::all(0), 2);
                cv::putText(frame, camera.getStringFromType(c.type), Point(int(c.x), int(c.y + 10)), 1, 1,
                            Scalar::all(0),
                            2);
            }
        }

//        SQUARES
        std::vector<CameraThread::Tree> healthy_trees;
        auto squares = camera.findHealthyTrees(frame, position.relative_altitude_m);
        for (auto sq: squares) {
            auto M = moments(sq.contour);
            double cX = M.m10 / M.m00;
            double cY = M.m01 / M.m00;

            auto treeGPS = camera.calculateGPSPosition(Point(cX, cY), sq.type, position, heading_deg);

            bool isValid = true;
            for (auto cts: circlesToShoot_GPS) {
                if (camera.distanceBetweenGPSPositions_m(cts, treeGPS) < 1) {
                    isValid = false;
                }
            }

            if (isValid) healthy_trees.push_back(treeGPS);
        }

        drawContours(frame, squares, -1, Scalar::all(255));

        std::cout << "PHOTO " << i << std::endl;
        for (auto c: circlesToShoot_GPS) {
            std::cout << camera.getStringFromType(c.type) << " [" << c.lat << ", " << c.lon << "]" << std::endl;
        }
        for (auto c: healthy_trees) {
            std::cout << camera.getStringFromType(CameraThread::TreeType::healthy_tree) << " [" << c.lat << ", "
                      << c.lon << "]" << std::endl;
        }

        cv::imwrite("../test/photo_" + std::to_string(i) + ".jpg", frame);

        cv::imshow("video", frame);

        switch (cv::waitKey(1)) {
            case 'q':
                goto exit;
            default:
                continue;
        }
    }
    exit:

    return 0;
}