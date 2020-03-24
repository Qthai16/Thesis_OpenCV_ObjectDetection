# Thesis_OpenCV_ObjectDetection
Building a Self-driving vehicle using OpenCV and HOG-SVM


## Overview

This project focus on lane detection using image processing and detect 3 types of traffic sign: Turn right, turn left and stop sign. The robot will try to move in the road and turn or stop if it detects a traffic sign.
This project is written in C++, using the OpenCV library for lane detection and HOG + SVM for object detection.

## Algorithm

These instructions will get you a copy of the project up and running on your local machine for development and testing purposes. See deployment for notes on how to deploy the project on a live system.

### Hardware connection
<p align="center">
  <img width="400" height="420" src="/img/hardware_connection.png">
</p>

### Lane detection and traffic signs classification algorithm
<p align="center">
  <img width="672" height="475" src="https://github.com/Qthai16/Thesis_OpenCV_ObjectDetection/blob/master/img/algorithm_all.png">
</p>

### Dependencies
In order to run the project, the following dependencies are required:
- Ubuntu 16.04 [here](http://cdimage.ubuntu.com/netboot/16.04/?_ga=2.243318149.1855666904.1529366501-828848615.1529366501)
- CMake minimum version 2.8 [here]
- OpenCV version xxx [here]
- Dlib library for object detection version xxx [here]
```
Give examples
```

### Build project and run

This project support 2 build options: Build on host (local PC) or build on Raspberry Pi.
To build on PC, open CMakeLists.txt and change BUILD_ON_PI/BUILD_ON_HOST to OFF/ON. Do the opposite when building on pi.

To build this project, do as below

```
cmake .
cd build && make
```

The building process takes approximately 5 mins in the first build (depend on your PC).

To run the project

```
./main
```

## Result

### GUI
<p align="center">
  <img width="547" height="529" src="https://github.com/Qthai16/Thesis_OpenCV_ObjectDetection/blob/master/img/imshow.png">
</p>

### HOG feature
<p align="center">
  <img width="700" height="242" src="https://github.com/Qthai16/Thesis_OpenCV_ObjectDetection/blob/master/img/HOG_feature.png">
</p>

You can find the demo video here: https://youtu.be/J12U36kCqYc
