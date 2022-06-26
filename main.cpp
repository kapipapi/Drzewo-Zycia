#include <iostream>
#include <future>
#include <thread>

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <mavsdk/plugins/mission_raw/mission_raw.h>
#include "cameraThread.cpp"

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
    if (argc < 3) {
        std::cout
                << "./main [connection string - sim: udp://:14540] [.plan file with mission: ../missions/drzewo_zycia_v1.plan]";
        return -1;
    }

    std::string connectionString = argv[1];
    std::string missionPath = argv[2];

    const float SHOOTING_HDG = 0.0;

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
    auto action = Action{system};
    auto mission_raw = MissionRaw{system};
    auto camera = CameraThread{&telemetry, ""};

    // We want to listen to the altitude of the drone at 1 Hz.
    const auto set_rate_result = telemetry.set_rate_position(1.0);
    if (set_rate_result != Telemetry::Result::Success) {
        std::cerr << "Setting rate failed: " << set_rate_result << '\n';
        return 1;
    }

    // Set up callback to monitor altitude while the vehicle is in flight
    telemetry.subscribe_position([](Telemetry::Position position) {
        std::cout << "Altitude: " << position.relative_altitude_m << " m\n";
    });

    std::cout << "Video capture initialization.\n";

    namedWindow("camera", WINDOW_NORMAL);
    std::thread camera_thread(CameraThread::RunThread, &camera);
    camera.subscribe_camera_output([&](const CameraThread::CameraOutput &output) {
        if (!output.circlesToShoot.empty()) {
            mission_raw.pause_mission();
            std::cout << "Mission paused.\n";

            for (const auto &cts: output.circlesToShoot) {
                sleep_for(seconds(1));

                printf("circle around [%.8f, %.8f] type:%d\n", cts.lat, cts.lon, cts.type);

                auto absolute_altitude = telemetry.position().absolute_altitude_m;

                auto goto_result = action.goto_location(cts.lat, cts.lon, absolute_altitude, SHOOTING_HDG);
                if (goto_result != Action::Result::Success) {
                    std::cout << "Goto circle failed: " << goto_result << '\n';
                    continue;
                }

                sleep_for(seconds(5));

                // GET NEW FRAME TO PRECISE CIRCLE POSITION
                std::vector<CameraThread::Tree> treePositionGroupToMean;
                for (int i = 0; i < 10; i++) {
                    UMat next = camera.getFreshFrame();

                    auto position = telemetry.position();
                    auto heading = telemetry.heading().heading_deg;
                    auto detectedCircles = camera.getCirclesInImage(next,
                                                                    position.relative_altitude_m);
                    auto detectedCirclesGPS = camera.circlesToGPSPositions(detectedCircles, position, heading);
                    auto newCirclesToShoot = camera.filterAlreadyShootedCircles(detectedCirclesGPS);

                    for (auto ncts: newCirclesToShoot) {
                        if (camera.distanceBetweenGPSPositions_m(cts, ncts) < 1) {
                            treePositionGroupToMean.push_back(ncts);
                        }
                    }

//                    imshow("camera", next);
                    sleep_for(milliseconds(100));
                }

                CameraThread::Tree meanPosition = camera.calculateMeanPosition(treePositionGroupToMean);
                if (!treePositionGroupToMean.empty()) {
                    printf("circle mean position [%.8f, %.8f] type:%d\n", meanPosition.lat, meanPosition.lon,
                           meanPosition.type);

                    auto goto_2_result = action.goto_location(meanPosition.lat, meanPosition.lon, absolute_altitude,
                                                              SHOOTING_HDG);
                    if (goto_2_result != Action::Result::Success) {
                        std::cout << "Goto circle failed: " << goto_2_result << '\n';
                        continue;
                    }

                    sleep_for(seconds(3));

                    // SHOOT
                    std::cout << "SHOOT DAMN" << std::endl;

                    sleep_for(seconds(1));

                    camera.treesAlreadyShot.push_back(meanPosition);
                }

                mission_raw.start_mission();
                std::cout << "Mission resumed.\n";
            }
        }
    });

    std::cout << "Camera ready.\n";

// Check until vehicle is ready to arm
    while (!telemetry.health_all_ok()) {
        std::cout << "Vehicle is getting ready to arm\n";
        sleep_for(seconds(1));
    }

    std::cout << "System ready\n";
    std::cout << "Creating and uploading mission\n";

    std::cout << "Importing mission from mission plan: " << missionPath << '\n';
    std::pair<MissionRaw::Result, MissionRaw::MissionImportData> import_res = mission_raw.import_qgroundcontrol_mission(
            missionPath);
    if (import_res.first != MissionRaw::Result::Success) {
        std::cerr << "Failed to import mission items: " << import_res.first;
        return 1;
    }

    if (import_res.second.mission_items.empty()) {
        std::cerr << "No missions! Exiting...\n";
        return 1;
    }
    std::cout << "Found " << import_res.second.mission_items.size() << " mission items in the given QGC plan.\n";

    std::cout << "Uploading mission...";
    const MissionRaw::Result upload_result = mission_raw.upload_mission(import_res.second.mission_items);
    if (upload_result != MissionRaw::Result::Success) {
        std::cerr << "Failed uploading mission: " << upload_result << '\n';
        return 1;
    }
    std::cout << "Mission uploaded.\n";

    std::cout << "Arming...\n";
    const Action::Result arm_result = action.arm();
    if (arm_result != Action::Result::Success) {
        std::cerr << "Arming failed: " << arm_result << '\n';
        return 1;
    }
    std::cout << "Armed.\n";

    auto missionProgressPromise = std::promise<bool>{};
    auto missionProgressFuture = missionProgressPromise.get_future();

// Before starting the mission subscribe to the mission progress.
    mission_raw.subscribe_mission_progress([&missionProgressPromise](
            MissionRaw::MissionProgress mission_progress
    ) {
        std::cout << "Mission progress update: " << mission_progress.current << " / " << mission_progress.total << '\n';
        if (mission_progress.current == mission_progress.total) {
            missionProgressPromise.set_value(true);
        }
    });

    const MissionRaw::Result start_mission_result = mission_raw.start_mission();
    if (start_mission_result != MissionRaw::Result::Success) {
        std::cerr << "Starting mission failed: " << start_mission_result << '\n';
        return 1;
    }

// wait for last mission point to be completed
    while (missionProgressFuture.valid() && !missionProgressFuture.get()) {}

// Mission complete. Command RTL to go home.
    std::cout << "Commanding RTL...\n";
    const Action::Result result = action.return_to_launch();
    if (result != Action::Result::Success) {
        std::cerr << "Failed to command RTL: " << result << '\n';
        return 1;
    }
    std::cout << "Commanded RTL.\n";

    auto landingActionPromise = std::promise<void>{};
    auto landingActionFuture = landingActionPromise.get_future();

    telemetry.subscribe_landed_state([&landingActionPromise](Telemetry::LandedState result) {
        if (result == Telemetry::LandedState::OnGround) {
            landingActionPromise.set_value();
        }
    });

// wait for landing complete
    landingActionFuture.get();

    telemetry.subscribe_landed_state(nullptr);

    std::cout << "Landed. Thanks for flying with WUThrust.\n";

    camera.stop();

    camera_thread.join();

    return 0;
}
