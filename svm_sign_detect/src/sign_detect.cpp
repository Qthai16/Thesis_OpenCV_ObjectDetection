#include "sign_detect.h"

SignDetect::SignDetect()
{

}
SignDetect::~SignDetect()
{

}

std::vector<TrafficSign> SignDetect::signs_data;
std::vector<object_detector<image_scanner_type> > SignDetect::detectors_data;
std::vector<int16_t> SignDetect::detect_index;

void SignDetect::Load_SVM_signs_detector(void) //** Load SVM detectors **//
{
	
	object_detector<image_scanner_type> tmp_detector;

	cout << "Loading SVM detectors..." << endl;

	// signs_data.push_back( TrafficSign ("right", "../svm_sign_detect/svm_detectors/turn_right_ver2.svm", rgb_pixel(255, 0, 0), 1  ));
	// signs_data.push_back( TrafficSign ("left" , "../svm_sign_detect/svm_detectors/turn_left_ver3.svm" , rgb_pixel(0, 0, 255), -1 ));
	// signs_data.push_back( TrafficSign ("stop" , "../svm_sign_detect/svm_detectors/stop_ver1.svm"      , rgb_pixel(0, 255, 0), 0  ));
	signs_data.push_back( TrafficSign ("right", "../svm_sign_detect/svm_detectors/new_right.svm", rgb_pixel(255, 0, 0), 1  ));
	signs_data.push_back( TrafficSign ("left" , "../svm_sign_detect/svm_detectors/new_left.svm" , rgb_pixel(0, 0, 255), -1 ));
	signs_data.push_back( TrafficSign ("stop" , "../svm_sign_detect/svm_detectors/new_stop.svm" , rgb_pixel(0, 255, 0), 0  ));
	
	for (int i = 0; i < signs_data.size(); i++) {
		deserialize(signs_data[i].svm_path) >> tmp_detector;
		detectors_data.push_back(tmp_detector);
	}
	// cout << "detector size: " << detectors.size() << endl;
	cout << "Finish loaded SVM detectors" << endl;
}

void SignDetect::sign_detect_update(const cv::Mat &src)
{
    // bool detect_flg = true;
    // cv::Rect draw_box;
    std::vector <dlib::rect_detection> rects;

    dlib::cv_image<dlib::bgr_pixel> images_HOG(src);
    // rects.clear();
    // draw_box.empty();
    detect_index.clear();
    evaluate_detectors(detectors_data, images_HOG, rects);

    for (int i=0;i<rects.size();i++)
    {
    	// cout << "confidence: " << rects[i].detection_confidence << endl;
        detect_index.push_back(signs_data[rects[i].weight_index].sign_index);
	}

#ifdef SHOW_DETECT
	cv::imshow("Detect signs", src);
#endif
}
