# Thesis_OpenCV_ObjectDetection
Building a Self-driving vehicle using OpenCV and HOG-SVM


## Overview

This project focus on lane detection using image processing and detect 3 types of traffic sign: Turn right, turn left and stop sign. The robot will try to move in the road and turn or stop if it detects a traffic sign.
This project is written in C++, using the OpenCV library for lane detection and HOG + SVM for object detection.

## Algorithm

These instructions will get you a copy of the project up and running on your local machine for development and testing purposes. See deployment for notes on how to deploy the project on a live system.

### Hardware connection
<p align="center">
  <img width="532" height="562" src="https://github.com/Qthai16/Thesis_OpenCV_ObjectDetection/tree/master/img/hardware_connection.png">
</p>

### Lane detection and traffic signs classification algorithm
<p align="center">
  <img width="840" height="594" src="https://github.com/Qthai16/Thesis_OpenCV_ObjectDetection/blob/master/img/algorithm_all.png">
</p>

### Dependencies
In order to run the project, the following dependencies are required:
- Ubuntu 16.04 [here](http://cdimage.ubuntu.com/netboot/16.04/?_ga=2.243318149.1855666904.1529366501-828848615.1529366501)
- CMake minimum version xxx [here]
- OpenCV version xxx [here]
- Dlib library for object detection version xxx [here]
```
Give examples
```

### Build project and run

A step by step series of examples that tell you how to get a development env running

Say what the step will be

```
Give the example
```

And repeat

```
until finished
```

End with an example of getting some data out of the system or using it for a little demo

## Result

### GUI
<p align="center">
  <img width="840" height="594" src="https://github.com/Qthai16/Thesis_OpenCV_ObjectDetection/blob/master/img/imshow.png">
</p>
### HOG feature
<p align="center">
  <img width="840" height="594" src="https://github.com/Qthai16/Thesis_OpenCV_ObjectDetection/blob/master/img/HOG_feature.png">
</p>
* You can find the demo video here: https://youtu.be/J12U36kCqYc
