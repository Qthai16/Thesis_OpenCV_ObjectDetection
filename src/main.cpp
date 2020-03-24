#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include <vector>
#include <math.h>
#include <time.h>

#include "../inc/pi_lane_detect.h"
#include "../inc/control.h"
#include "../inc/utility.h"
#include "../svm_sign_detect/src/sign_detect.h"

#include "../inc/raspi_uart.h"

#include <pthread.h>

using namespace dlib;

#define _DEBUG_
// #define SHOW_FPS
// #define STREAM
// #define SAVE_VID
// #define TEST_SVM

#define FRAME_WIDTH     320
#define FRAME_HEIGHT    240

// #define FRAME_WIDTH_RESIZED     400
// #define FRAME_HEIGHT_RESIZED    300
// #define FRAME_WIDTH     640
// #define FRAME_HEIGHT    480
#define VID_SRC 0

#define SEND_DATA
// #define RECV_DATA

DetectLane *lane_detect;
Control *car_control;
PI_UART *pi_uart;

Mat src;

const string Video_Path_Host = "/home/qthai/Downloads/Code_shorten_SVM_7_5/Vid_no_SVM_3.avi";
const string Video_Path_Pi = "/home/pi/Code_new/Vid_port_jetson_17_5.avi";

uint8_t u8_Buf[100];
uint16_t u16_data_send[2];
uint8_t u8_save_txt_buf[100];

// int flg = 2; //binh thuong, di thang

Scalar minHSV_lft_rgt_sign = Scalar(78, 66, 0);
Scalar maxHSV_lft_rgt_sign = Scalar(132, 255, 255);

Scalar minHSV_stop_sign = Scalar(0,136,92);
Scalar maxHSV_stop_sign = Scalar(255,255,255);

// Scalar minHSV_stop_sign = Scalar(0,155,92);
// Scalar maxHSV_stop_sign = Scalar(209,255,255);


int16_t volatile found_sign_main = 0;
Mat sign_crop;
int16_t which_sign_flg = 2;

bool volatile mat_lock = false;

cv::Mat pre_process_svm(const cv::Mat &src);

cv::Mat pre_process_svm(const cv::Mat &src)
{
    Mat src_hsv;
    Mat src_inrange_lft_rgt, src_inrange_stop;
    Mat src_crop;

    Mat src_nothing = Mat::zeros(Size(50,50), CV_8UC3);
    Rect roi;
    int x_rect=0, y_rect=0, width_r=0, height_r=0;
    cv::Point tl_rect = Point(-1, -1);
    cv::Point br_rect = Point(-1, -1);

    cvtColor(src, src_hsv, COLOR_BGR2HSV);

    // inRange(src_hsv, Scalar(minHSV[0], minHSV[1], minHSV[2]) , Scalar(maxHSV[0], maxHSV[1], maxHSV[2]), src_inrange);
    inRange(src_hsv, minHSV_lft_rgt_sign , maxHSV_lft_rgt_sign, src_inrange_lft_rgt);
    inRange(src_hsv, minHSV_stop_sign    , maxHSV_stop_sign   , src_inrange_stop);
    Mat mask_combine = src_inrange_lft_rgt + src_inrange_stop;

    std::vector<std::vector<cv::Point> > contours; //Tao 1 vector 2 chieu voi moi phan tu la Point
    std::vector<Vec4i> hierarchy;
    findContours(mask_combine, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0));

    std::vector<cv::Rect> boundRect( contours.size() );
    std::vector<std::vector<Point>> contours_poly( contours.size() );

    float max_rect_area = 0;
    size_t max_rect_ind = 0;

    // Mat drawing = Mat::zeros(mask_combine.size(), CV_8UC1);
    for (int i = 0; i < contours.size(); i++)
    {
        approxPolyDP( Mat(contours[i]), contours_poly[i], 3, false );
        // drawContours(drawing, contours, i, 255, 2, 8, hierarchy, 0, Point());
        int area = contourArea(contours_poly.at(i));
        if (area > 200) //loai bo contour nhieu
            boundRect[i] = boundingRect( Mat(contours_poly[i]) );

        float tmp_rect_area = (boundRect[i].width)*(boundRect[i].height);
        if (tmp_rect_area > max_rect_area){ //lay contour co dien tich lon nhat
            if ( abs( boundRect[i].width - boundRect[i].height ) < 0.2*(boundRect[i].width) )// contour gan vuong
            { 
                max_rect_area = tmp_rect_area;
                max_rect_ind = i;
            }
        }
        // cv::rectangle( src, boundRect[i].tl(), boundRect[i].br(), Scalar(255,0,0), 2, 8, 0 );
    }
    // cv::rectangle(src, boundRect[max_rect_ind].tl(), boundRect[max_rect_ind].br(), Scalar(0,0,255), 4, 8, 0 );
    // imshow("drawing", drawing);

    if (boundRect.size() > 0)
    {
        tl_rect = boundRect[max_rect_ind].tl();
        br_rect = boundRect[max_rect_ind].br();
        width_r  = br_rect.x - tl_rect.x+ 40;
        height_r = br_rect.y - tl_rect.y+ 40;

        x_rect = tl_rect.x - 20;
        y_rect = tl_rect.y - 20;
        // width_r = width_r + 20;
        // height_r = height_r + 20;

        if (x_rect < 0)
            x_rect = 0;
        if (y_rect < 0)
            y_rect = 0;
        if (width_r > FRAME_WIDTH)
            width_r = FRAME_WIDTH;
        if (height_r > FRAME_HEIGHT)
            height_r = FRAME_HEIGHT;
        roi = Rect(x_rect, y_rect,  width_r, height_r);

        if(roi.x >= 0 && roi.y >= 0 && roi.width + roi.x < FRAME_WIDTH && roi.height + roi.y < FRAME_HEIGHT)
        {
            // while (mat_lock);
            // mat_lock = true;

            src_crop = src(roi).clone();
            // Mat src_crop_tmp = src(roi);
            // src_crop_tmp.copyTo(src_crop);
            // mat_lock = false;
        }
    }

    if ( (src_crop.rows > 0) && (src_crop.cols > 0) ){
        found_sign_main = 1;
        // return src_crop.clone();
        return src(roi);
    }
    else{
        found_sign_main = 0;
    	return src_nothing;
    }
}

void* get_blah_blah(void* arg )
{
    SignDetect *sign_detect;
    sign_detect = new SignDetect();
    // int16_t *p_found_flg = (int16_t *)arg;
    sign_detect->Load_SVM_signs_detector();
    while(1)
    {
        // int16_t p_found_flg = *((int16_t *)arg);
        // cout << "OK" << endl;
        // cout << *p_found_flg << endl;
        if ( found_sign_main == 1)
        {
            // while(mat_lock);
            // mat_lock = true;
            sign_detect->sign_detect_update(sign_crop);
            // mat_lock = false;

            if((sign_detect->detect_index).size() == 0){
                // cout << "NF"; //<< endl;
                which_sign_flg = 3;
            }

            for (int i=0;i<(sign_detect->detect_index).size() ;i++)
            {
                if ( (sign_detect->detect_index)[i] == 1)//turn right ahead sign
                {
                    cout << "R" << endl;
                    which_sign_flg = 1;
                }
                else if ( (sign_detect->detect_index)[i] == -1)//turn left ahead sign
                {
                    cout << "L" << endl;
                    which_sign_flg = -1;
                }
                else if ( (sign_detect->detect_index)[i] == 0)//stop sign
                {
                    cout << "S" << endl;
                    which_sign_flg = 0;
                }
            }
            // cout << "OK 1" <<endl;  
        }

        else
        {
            which_sign_flg = 2;
        }
    }
}


int main(int argc, char **argv)
{
    bool playVideo = true;
#ifdef STREAM
    VideoCapture capture(Video_Path_Host);
#else
    VideoCapture capture(VID_SRC);
#endif

#ifdef SAVE_VID
    cv::VideoWriter video(Video_Path_Pi,CV_FOURCC('M','J','P','G'),30, Size(FRAME_WIDTH,FRAME_HEIGHT), true);
#endif

    lane_detect = new DetectLane();
    car_control = new Control();
    pi_uart = new PI_UART();
    
    car_control->pid_init();
    
	if( !capture.isOpened() )
        throw "Error when reading steam_avi";
        
    // capture.set(CV_CAP_PROP_FPS, 30);
    capture.set(CV_CAP_PROP_FRAME_WIDTH,FRAME_WIDTH);
    capture.set(CV_CAP_PROP_FRAME_HEIGHT,FRAME_HEIGHT);

#ifdef SHOW_FPS
    int num_frames = 0;
    time_t start, end;
    time(&start);
    Utility::initTime();
#endif

//** Sign Detection Processing Start Here **//
    sign_crop = Mat::zeros(Size(50,50), CV_8UC3);
	pthread_t thread;
	int rc;
	rc = pthread_create(&thread, NULL, get_blah_blah , NULL);
	// cout << "Sign Detection Thread Created" << endl;
    if (rc) {
        cout << "Error:unable to create thread," << rc << endl;
        exit(-1);
    }
//** Sign Detection Processing End Here **//

    while (true)
    {
        if (playVideo){
            capture >> src;
        }

        if (src.empty()){
            break;
        }

        // while(mat_lock);
        // mat_lock = true;
        // src_cp = src.clone();
        // mat_lock = false;

        sign_crop = pre_process_svm(src);
        // cout << "found_sign_main " << found_sign_main << endl;
        
#ifdef SAVE_VID
        video.write(src);
#endif
//** Image Processing Start Here **//
        // resize(src, src_resize, Size(320,240), 0, 0, INTER_LINEAR);
        lane_detect->Trackbar_Window();
        lane_detect->update(src);
//** Image Processing End Here  **//

//** Control Start Here         **//
        car_control->driverCar_ver2(lane_detect->center_lft_lane, lane_detect->center_rgt_lane, lane_detect->center_road_lane, which_sign_flg);
        car_control->my_pub(u16_data_send);
        // cout << "[" << u16_data_send[0] << "," << u16_data_send[1]  << "]" << endl;
//** Control End Here           **//

        // if (lane_detect->variance > 30){
        //     u16_data_send[1] = 40;
        // }
        // else {
        //     u16_data_send[1] = 35;
        // }
#ifdef TEST_SVM
        u16_data_send[0] = 78; //de test bien bao
#endif
        
#ifdef SEND_DATA
        sprintf((char*)(u8_Buf), "[%d,%d]", u16_data_send[0], u16_data_send[1]);
        pi_uart->uart_string_send(u8_Buf);
#endif

#ifdef RECV_DATA
        pi_uart->uart_string_recv(u8_Buf);
#endif

#ifdef SHOW_FPS
        ++num_frames;
        time(&end);
        double seconds = difftime(end, start);
        if (seconds >= 1.0)
        {
           cout << "FPS: " << num_frames << endl;//<< " Time: " << seconds << endl;
           time(&start);
           num_frames = 0; 
        }
#endif

#ifdef _DEBUG_
        cv::imshow("View", src);
        // imshow("crop sign", sign_crop);
        char key = cv::waitKey(1)&0xFF;
        if(key == 'p')
            playVideo = !playVideo; 
        else if (key == 'q')
            break;
#endif
    }
    
    capture.release();
	cv::destroyAllWindows();
	return 0;
}
