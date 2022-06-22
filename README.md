# Drzewo-Zycia
Konkurencja Droniada 2022

## Dependencies
- [OpenCV](https://github.com/opencv/opencv)
- [MAVSDK](https://mavsdk.mavlink.io)

## Usefull commands

### run PX4 simulation with homepoint in warsaw:
`HEADLESS=1 PX4_HOME_LAT=52.1672117 PX4_HOME_LON=20.99803 make px4_sitl gazebo`

## Bugs
- [ ] circle waypoint generated with wrong latitude

## TODO
- [ ] function (drone height) => circle radius (min, max)
- [x] ROI circle
- [ ] ROI circle average color
- [ ] filter out circles with color
- [ ] calculateGPSPosition - add calculation for circle center-photo center offset
