//
// Created by kacper on 25.06.22.
//
#include <iostream>
#include <fstream>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>

#include "../../cameraThread.cpp"

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
    camera.DRAWING = false;

    camera.cap.open(0);
    if (!camera.cap.isOpened()) {
        std::cout << "Camera capture is not open!" << std::endl;
        return 1;
    }

    std::vector<CameraThread::Tree> alreadyDetected;

    std::ofstream file("output.csv");

    while (true) {
        cv::UMat frame;// = camera.getFreshFrame();
        camera.cap >> frame;
        auto position = telemetry.position();
        auto heading_deg = telemetry.heading().heading_deg;

        if (frame.empty()) {
            continue;
        }

//        CIRCLES
        auto detectedCircles = camera.getCirclesInImage(frame, position.relative_altitude_m);
        auto detectedCircles_GPS = camera.circlesToGPSPositions(detectedCircles, position, heading_deg);
        std::vector<CameraThread::Tree> treesToShoot = camera.filterAlreadyShootedCircles(detectedCircles_GPS);
        for (auto tts: treesToShoot) {
            if (tts.type != CameraThread::probably_grass) {
                bool isAlreadyDetectedTree = false;
                for (auto adt: alreadyDetected) {
                    if (camera.distanceBetweenGPSPositions_m(tts, adt) < 1) {
                        isAlreadyDetectedTree = true;
                        break;
                    }
                }
                if (!isAlreadyDetectedTree) {
                    alreadyDetected.push_back(tts);
                }
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

            bool isAlreadyDetectedTree = false;
            for (auto adt: alreadyDetected) {
                if (camera.distanceBetweenGPSPositions_m(adt, treeGPS) < 1) {
                    isAlreadyDetectedTree = true;
                }
            }

            if (!isAlreadyDetectedTree) {
                alreadyDetected.push_back(treeGPS);

            }
        }

        for (auto c: alreadyDetected) {
            std::cout << camera.getStringFromType(c.type) << " [" << c.lat << ", " << c.lon << "]" << std::endl;
            file << c.lat << ", " << c.lon << ", " << camera.getStringFromType(c.type) << std::endl;
        }

        file << "dupa" << std::endl;

        sleep_for(milliseconds(30));
    }

    return 0;
}