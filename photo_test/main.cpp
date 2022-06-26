//
// Created by kacper on 25.06.22.
//

//
// Created by kacper on 25.06.22.
//

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>

#include "../cameraThread.cpp"

CameraThread::Tree getGPS(CameraThread camera, Point circenter, Telemetry::Position droneposition, double heading) {
    return camera.calculateGPSPosition(circenter, CameraThread::pathogen_beige_tree,
                                       droneposition, heading);
}


int main() {

    CameraThread camera{};
    camera.pixel_calculation_error_rate = 0.3;
    camera.focal_length_px = 16867.768;
    camera.imageWidth = 8192;
    camera.imageHeight = 5460;

    auto tree1 = getGPS(camera, Point(3683, 2257), {50.238846704, 19.027875130, 0, 32.068}, 0);
    auto tree2 = getGPS(camera, Point(647, 3566), {50.238953731, 19.027997820, 0, 31.971}, 0);

    std::cout.precision(15);
    std::cout << tree1.lat << ", " << tree1.lon << std::endl;
    std::cout << tree2.lat << ", " << tree2.lon << std::endl;

    return 0;
}