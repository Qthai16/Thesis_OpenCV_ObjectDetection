#ifndef SIGN_DETECT_H
#define SIGN_DETECT_H

#include <dlib/svm_threaded.h>
#include <dlib/string.h>
#include <dlib/gui_widgets.h>
#include <dlib/image_processing.h>
#include <dlib/data_io.h>
#include <dlib/image_transforms.h>
#include <dlib/cmd_line_parser.h>

#include <dlib/opencv.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <time.h>
#include <iostream>
#include <fstream>
#include <cstdlib>

using namespace std;
using namespace dlib;
using namespace cv;

// #define SHOW_DETECT

typedef scan_fhog_pyramid<pyramid_down<6> > image_scanner_type;
// Get the upsample option from the user but use 0 if it wasn't given.

struct TrafficSign {
  string name;
  string svm_path;
  rgb_pixel color;
  int16_t sign_index;
  TrafficSign(string name, string svm_path, rgb_pixel color, int16_t sign_index) :
    name(name), svm_path(svm_path), color(color), sign_index(sign_index) {};
};

class SignDetect
{
public:
    SignDetect();
    ~SignDetect();

	static std::vector<TrafficSign> signs_data;
	static std::vector<dlib::object_detector<image_scanner_type> > detectors_data;
	// static cv::Rect draw_box;
    // static std::vector <dlib::rect_detection> rects;
	static std::vector<int16_t> detect_index; //Nhan biet toi da 3 bien bao
	
	void Load_SVM_signs_detector(void);
	void sign_detect_update(const cv::Mat &src);

private:

};



#endif
