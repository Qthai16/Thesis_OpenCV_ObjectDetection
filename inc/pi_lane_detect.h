#ifndef PI_LANE_DETECT_H
#define PI_LANE_DETECT_H

#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include <vector>
#include <math.h>
#include <algorithm>

#include "utility.h"

using namespace std;
using namespace cv;

// #define CALIB
#define SHOW_WINDOW

class DetectLane
{
public:
    DetectLane();
    ~DetectLane();

    static int slideThickness;
    static int VERTICAL;
    static int HORIZONTAL;

    static int BIRDVIEW_WIDTH;
    static int BIRDVIEW_HEIGHT;

    static Point null;
    static int LEFT_LANE;
    static int RIGHT_LANE;

    static double variance ;
    static Point mass_road_src;
    static Point center_lft_lane, center_rgt_lane;
    static Point center_road_lane;
    static bool turning_ahead;

    void update(const Mat &src);
    
    vector<Point> getLeftLane();
    vector<Point> getRightLane();
    void Trackbar_Window(void);
    int16_t lostLeftLane_flg = -1, lostRightLane_flg = -1;

private:
    float lft_deviant_ang = -105.0, rgt_deviant_ang =  -68.0;
    int half_laneWidth = 60;

//** Morphological function
    int filter_kernel_size = 3;
    int erode_kernel_size = 1;
    int dilate_kernel_size = 2;
//** sobelfilter function
    float alpha_addweight_sobfunc = 0.0;
    float beta_addweight_sobfunc = 1.0;
    int bin_threshold_sobfunc = 80;
//** Nguong bien duong (lane) **//
    // int minHSV[3] = {34, 74, 98};
    // int maxHSV[3] = {114, 255, 255};
    int minHSV[3] = {0, 0, 0};
    int maxHSV[3] = {255, 255, 180};

    Scalar minHSV_lane = Scalar(50, 35, 118); //Nguong nay tam duoc (buoi sang)
    Scalar maxHSV_lane = Scalar(255, 255, 255);

    // Scalar minHSV_lane = Scalar(50,80,110); // Nguong nay buoi toi
    // Scalar maxHSV_lane = Scalar(255, 255, 255);

    // Scalar minHSV_lane = Scalar(0, 35, 100);
    // Scalar maxHSV_lane = Scalar(255, 255, 255);
//** Nguong tam duong (centroid) **//
    Scalar minHSV_massofroad = Scalar(0, 0, 0);
    Scalar maxHSV_massofroad = Scalar(255, 255, 180);
    // Scalar maxHSV_massofroad = Scalar(255, 255, 84);

    int offset_y = 85; //loai bo phia tren duong chan troi
    // int offset_y = 70;
    // int offset_y = 90;

    vector<Point> leftLane, rightLane;

    Mat preProcess( const Mat &src, Point& mass_road_BirdView );
    Mat sobelfilter(const Mat &src_rgb);
    void morphOps(Mat &thresh, int* erode_kernel_size, int* dilate_kernel_size);

    Mat birdViewTranform( const Mat &source, Point& mass_road );
    //** Chuyen doi goc nhin sang BirdView su dung Prespective Transform **//
    Point MassOfRoad(const Mat &src_rgb);
    Point detectMassRoad( vector< vector<Point> >& contours );
    //** Tinh tam duong trong anh goc (chua chuyen goc nhin) **//
    

    void fillLane( Mat &src );
    //** Ve lai duong lane su dung thuat toan Hough Line Points **//
    vector<Mat> splitLayer( const Mat &src, int dir = VERTICAL );
    //** Cat anh goc (src) thanh N anh nho hon theo chieu ngang (horizontal) hoac doc (vertical) **//
    vector< vector<Point> > centerRoadSide( const vector<Mat> &src, int dir = VERTICAL );

    void detectLeftRight( const vector< vector<Point> > &points, Point& mass_road );
    //** Xap xi 2 lane duong bang N diem (toi da 32 diem), xac dinh lane trai, phai, goc lech lane(khi sap vao cua), variance **//
    int recognize_left_right(vector<Point> &lanex, Point &mass_road, double& var);
    //** Nhan biet lane trai, lane phai dua vao BirdView, tam duong **//
    int16_t Detect_one_lane( vector<Point> &lanex, Point &mass_road, double& var, float ang_1, float ang_2);
    //** Nhan biet 1 lane dua vao goc lech duong thang xap xi, tra ve -1 thi lanex la lane trai, tra ve 1 thi lanex la lane phai **//
    int16_t Lost_lane(const vector<Point> &points);
    //** Nhan biet mat lane, neu tra ve -1 tuc la mat lane, neu tra ve 0 tuc la co lane **//
    bool point_in_rect( Rect rect_win, Point p);
    //** Nhan biet 1 diem co nam trong Rect hay khong, tra ve true neu In Rect va Nguoc lai

};

#endif
