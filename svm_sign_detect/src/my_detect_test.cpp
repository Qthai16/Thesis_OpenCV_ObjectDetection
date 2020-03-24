#include <iostream>
#include <stdlib.h>
#include <math.h>

#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <time.h>
#include <fstream>
#include <cstdlib>

#include "sign_detect.h"

using namespace std;
using namespace dlib;
using namespace cv;

#define VID_SRC 0
// #define CALIB

SignDetect *detect;

int minHSV[3] = {0, 0, 0};
int maxHSV[3] = {255, 255, 255};
// int edge_thresh = 50;

Scalar minHSV_lft_rgt_sign = Scalar(78, 66, 0);
Scalar maxHSV_lft_rgt_sign = Scalar(132, 255, 255);

Scalar minHSV_stop_sign = Scalar(0,136,177);
Scalar maxHSV_stop_sign = Scalar(255,255,255);

void Trackbar_Window(void);

void Trackbar_Window(void)
{
#ifdef CALIB
    namedWindow("Threshold", 0);
    createTrackbar("LowH", "Threshold", &minHSV[0], 255);
    createTrackbar("HighH", "Threshold", &maxHSV[0], 255);

    createTrackbar("LowS", "Threshold", &minHSV[1], 255);
    createTrackbar("HighS", "Threshold", &maxHSV[1], 255);

    createTrackbar("LowV", "Threshold", &minHSV[2], 255);
    createTrackbar("HighV", "Threshold", &maxHSV[2], 255);
    // createTrackbar("Edged Thresh", "Threshold", &edge_thresh, 110);
#endif
}

int main(int argc, char** argv) {
	VideoCapture capture(0);
	capture.set(CV_CAP_PROP_FPS, 30);
	capture.set(CV_CAP_PROP_FRAME_WIDTH,320);
	capture.set(CV_CAP_PROP_FRAME_HEIGHT,240);
	Mat src;
	Mat gray, src_hsv;
	Mat src_inrange_lft_rgt, src_inrange_stop;
	Mat edged;
	Mat and_mask = Mat::zeros(Size(320,240), CV_8UC3);
	std::vector<Vec3f> circles;



	Trackbar_Window();

	detect = new SignDetect();
	detect->Load_SVM_signs_detector();

	while (true)
	{
		if( !capture.isOpened() )
        	throw "Error when reading steam_avi";
        
	    capture >> src;

	    Mat src_cp = src.clone();
	    Mat src_cp_hough = src.clone();

	    if (src.empty()){
	        break;
	    }

		cvtColor(src, src_hsv, COLOR_BGR2HSV);

	    // inRange(src_hsv, Scalar(minHSV[0], minHSV[1], minHSV[2]) , Scalar(maxHSV[0], maxHSV[1], maxHSV[2]), src_inrange);
	    inRange(src_hsv, minHSV_lft_rgt_sign , maxHSV_lft_rgt_sign, src_inrange_lft_rgt);
	    inRange(src_hsv, minHSV_stop_sign    , maxHSV_stop_sign   , src_inrange_stop);
	    Mat mask_combine = src_inrange_lft_rgt + src_inrange_stop;
	    // Canny(src_inrange, edged, edge_thresh, edge_thresh*2, 3);
	    // std::cout << src_inrange.type() << ", " << src.type() << endl;
	    // bitwise_and(src, src_inrange, and_mask);

        std::vector<std::vector<cv::Point> > contours; //Tao 1 vector 2 chieu voi moi phan tu la Point
    	std::vector<Vec4i> hierarchy;
		findContours(mask_combine, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0));
		std::vector<cv::Rect> boundRect( contours.size() );

		float max_rect_area = 0;
		size_t max_rect_ind = 0;

		Mat drawing = Mat::zeros(mask_combine.size(), CV_8UC1);
		for (int i = 0; i < contours.size(); i++)
		{
			int area = contourArea(contours.at(i));
		    drawContours(drawing, contours, i, 255, 2, 8, hierarchy, 0, Point());

		    if (area > 300)
		    	boundRect[i] = boundingRect( Mat(contours[i]) );

		    float tmp_rect_area = (boundRect[i].width)*(boundRect[i].height);
		    if (tmp_rect_area > max_rect_area){
		    	if ( abs( boundRect[i].width - boundRect[i].height ) < 30){
			    	max_rect_area = tmp_rect_area;
			    	max_rect_ind = i;
		    	}
		    }
		    cv::rectangle( src_cp, boundRect[i].tl(), boundRect[i].br(), Scalar(255,0,0), 2, 8, 0 );
		}
		cv::rectangle(src_cp, boundRect[max_rect_ind].tl(), boundRect[max_rect_ind].br(), Scalar(0,0,255), 4, 8, 0 );

	    // Mat src_crop = Mat::zeros(src.size(), CV_8UC3 );
	    Mat src_crop = src(boundRect[max_rect_ind]);


	    //** Hough circle start here **//
	    /**
        cvtColor(src, gray, COLOR_BGR2GRAY);
    	// medianBlur(gray, gray, 5);
    	HoughCircles(gray, circles, HOUGH_GRADIENT, 1, gray.rows/16,  // change this value to detect circles with different distances to each other
                 													100, 30, 40, 70);
    	// change the last two parameters
        // (min_radius & max_radius) to detect larger circles
		for( size_t i = 0; i < circles.size(); i++ )
	    {
	        Vec3i c = circles[i];
	        Point center = Point(c[0], c[1]);
	        // circle center
	        circle( src_cp_hough, center, 1, Scalar(0,100,100), 3, LINE_AA);
	        // circle outline
	        int radius = c[2];
	        circle( src_cp_hough, center, radius, Scalar(255,0,255), 3, LINE_AA);
	    }
	    **/
	    //** Hough circle start here **//

	    
	    //** Detect which traffic sign start here **//
		if ( (src_crop.rows > 0) && (src_crop.cols > 0) ) //tim duoc nguong hsv voi contour lon nhat
		{
			imshow("sign crop", src_crop);
			detect->sign_detect_update(src_crop);

		    for (int i=0;i<(detect->detect_index).size() ;i++)
	    	{
	    		if ( (detect->detect_index)[i] == 1)//turn right ahead sign
			    {
			    	std::cout << "right" << endl;
			    }
			    else if ( (detect->detect_index)[i] == -1)//turn left ahead sign
			    {
			    	std::cout << "left" << endl;
			    }
			    else if ( (detect->detect_index)[i] == 0)//stop sign
			    {
			    	std::cout << "stop" << endl;
			    }
	    	}
		}
		//** Detect which traffic sign end here **//

	    // imshow("detected circles", src_cp_hough);
		// imshow("after inrange", mask_combine);
		// imshow("contour", drawing);
		// imshow("rect", src_cp);
		// imshow("and mask", and_mask);
	    
        char key = cv::waitKey(1)&0xFF;
        if (key == 'q')
            break;
	}
	cv::destroyAllWindows();
	return 0;
}


	    


	    





