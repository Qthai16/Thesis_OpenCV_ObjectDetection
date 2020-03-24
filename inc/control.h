#ifndef CONTROL_H
#define CONTROL_H

#include <stdio.h>
#include <stdint.h>
#include <iostream>
#include <vector>
#include <math.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include "pi_lane_detect.h"

using namespace std;
using namespace cv;

// #define KP 2
// #define KI 0.01
// #define KD 0.005
// #define T 0.04
#define PI	3.14159265
//** thong so quy doi can sua
#define GAIN_CVT_SMALL	1.0
#define BIAS_CVT_SMALL	78

#define GAIN_CVT_BIG	1.0
#define BIAS_CVT_BIG	78

//** thong so quy doi can sua

class Control
{
public:
	Control(); //Constructor
	~Control(); //Destructor
	
	typedef struct
	{
		float Kp;
		float Ki;
		float Kd;
		float u;
		float u_;
		float error;
		float error_;
		float error__;
		float Ts;
		float PID_Saturation;
	} PID_PARAMETERS;

	PID_PARAMETERS angle_crtl_pid;
	PID_PARAMETERS angle_small_ctrl_pid;
	int laneWidth = 100;
	int ctrl_half_laneWidth = 60;
	Point carPos;
	float Angle_Velo_data[2];
	float *p_data = Angle_Velo_data;
	
	int8_t pid_init(void);
	int8_t pid_set_para(PID_PARAMETERS* pid_parameter,float Kp,float Ki, float Kd);
	// void driverCar(const vector<Point> &left, const vector<Point> &right, float velocity, int flag);
	void driverCar_ver2(const Point &center_left, const Point &center_right, const Point &center_lane, int flag);
	void my_pub(uint16_t* data_buf);
	
private:
	float pre_angle;
	float current_angle;

	float compute_angle_error(const Point &dst);
	float pid_process_raw(PID_PARAMETERS* pid_parameter,double current_error);
	uint16_t Convert_to_DutyCycles(float udk, float angle);

};

#endif
