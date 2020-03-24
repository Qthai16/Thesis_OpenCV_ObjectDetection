#include "../inc/pi_lane_detect.h"

//splitLayer variable
int DetectLane::slideThickness = 10;
int DetectLane::VERTICAL = 0;
int DetectLane::HORIZONTAL = 1;

int DetectLane::BIRDVIEW_WIDTH = 240;
int DetectLane::BIRDVIEW_HEIGHT = 320;

Point DetectLane::null = Point(-1,-1);
int DetectLane::LEFT_LANE = 2;
int DetectLane::RIGHT_LANE = 3;

double DetectLane::variance = 0;
Point DetectLane::mass_road_src = null;
Point DetectLane::center_lft_lane = null;
Point DetectLane::center_rgt_lane = null;
Point DetectLane::center_road_lane = null;
// int16_t DetectLane::lft_rgt_flag = 0;
bool DetectLane::turning_ahead = false;
// Point DetectLane::mass_road_static = Point( BIRDVIEW_WIDTH/2, BIRDVIEW_HEIGHT/2 );

// int min(int a, int b)
// {
//     return a < b ? a : b;
// }

DetectLane::DetectLane()
{
    
}

void DetectLane::Trackbar_Window(void)
{
	#ifdef CALIB
    namedWindow("Threshold", 0);
    createTrackbar("LowH", "Threshold", &minHSV[0], 255);
    createTrackbar("HighH", "Threshold", &maxHSV[0], 255);

    createTrackbar("LowS", "Threshold", &minHSV[1], 255);
    createTrackbar("HighS", "Threshold", &maxHSV[1], 255);

    createTrackbar("LowV", "Threshold", &minHSV[2], 255);
    createTrackbar("HighV", "Threshold", &maxHSV[2], 255);

    createTrackbar( "Erode", "Threshold", &erode_kernel_size, 8 );
    createTrackbar( "Dilate", "Threshold", &dilate_kernel_size, 8 ); 
    createTrackbar("offset Y", "Threshold", &offset_y, 240);
	#endif
}

DetectLane::~DetectLane() {}

vector<Point> DetectLane::getLeftLane()
{
    return leftLane;
}

vector<Point> DetectLane::getRightLane()
{
    return rightLane;
}

void DetectLane::update(const Mat &src)
{
    Point mass_road_BV = null;
    // Mat birdView = Mat::zeros(Size(BIRDVIEW_WIDTH, BIRDVIEW_HEIGHT), CV_8UC3);
    Mat lane     = Mat::zeros(Size(BIRDVIEW_WIDTH, BIRDVIEW_HEIGHT), CV_8UC3);
    Mat bev_img  = Mat::zeros(Size(BIRDVIEW_WIDTH, BIRDVIEW_HEIGHT), CV_8UC3);
    //input: rgb ,output: binary image with massroad point

    Point mass_road_tmp = null;
    Mat dilateElement = getStructuringElement( MORPH_RECT,Size(2*(2) + 1,2*(2) + 1));
    Mat imgsobel = sobelfilter(src);
    
    mass_road_tmp = MassOfRoad(src); //chu y day la mass_road o trong anh goc

    mass_road_src.x = mass_road_tmp.x;
    mass_road_src.y = mass_road_tmp.y;

    bev_img = birdViewTranform(imgsobel, mass_road_tmp); //chu y day la mass road trong anh BirdView
    
    mass_road_BV.x = mass_road_tmp.x;
    mass_road_BV.y = mass_road_tmp.y;

    // Mat bev_img = preProcess(src, mass_road_BV); //sau khi qua preProcess thi massroad nay la massroad trong birdview
    // mass_road_src = MassOfRoad(src);

    //input: binary , ouput: vector<Mat> 
    vector<Mat> layers1 = splitLayer(bev_img);
    vector< vector<Point> > points1 = centerRoadSide(layers1);

    // birdView = Mat::zeros(Size(BIRDVIEW_WIDTH, BIRDVIEW_HEIGHT), CV_8UC3);
    // lane = Mat::zeros(Size(BIRDVIEW_WIDTH, BIRDVIEW_HEIGHT), CV_8UC3);
    
    detectLeftRight(points1, mass_road_BV); // return left and right point array

#ifdef SHOW_WINDOW
// ****** Scalar(x,y,z) : BLUE GREEN RED ******//
    for (int i = 0; i < points1.size(); i++) //mau trang chua biet lane nao
    {
        for (int j = 0; j < points1[i].size(); j++)
        {
            circle(lane, points1[i][j], 1, Scalar(255, 255, 255), 2, 8, 0);
        }
    }

    for (int i = 1; i < leftLane.size(); i++) //mau do lane trai
    {
        if (leftLane[i] != null)
        {
            circle(lane, leftLane[i], 1, Scalar(0, 0, 255), 2, 8, 0);
        }
    }

    for (int i = 1; i < rightLane.size(); i++) //mau xanh duong lane phai
    {
        if (rightLane[i] != null)
        {
            circle(lane, rightLane[i], 1, Scalar(255, 0, 0), 2, 8, 0);
        }
    }

    circle(lane, center_lft_lane, 4, Scalar(255,0,0), 2, 8, 0); //Tam lane trai, mau xanh duong
    circle(lane, center_rgt_lane, 4, Scalar(0,0,255), 2, 8, 0); //Tam lane phai, mau do
    circle(lane, center_road_lane, 3, Scalar(0,255,0), 2, 8, 0);//Tam de dieu khien, mau xanh la

    circle(lane, Point(BIRDVIEW_WIDTH/2, BIRDVIEW_HEIGHT-40), 1, Scalar(50, 255, 120), 2, 8, 0); //carpos
    circle(lane, mass_road_BV, 3, Scalar(0, 255, 0), 2, 8, 0); //tam duong trong BirdView mau xanh la
    circle(src, mass_road_src, 3, Scalar(50,127,255), -1, 8, 0); //tam duong trong src mau xanh la

    imshow("Lane Detect", lane);
#endif
}


////////////////////////** Preprocess, Mass of road, Bird View Transfrom Start here **////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

Mat DetectLane::preProcess(const Mat &src, Point &mass_road_BirdView)
{
    Point mass_road_tmp = null;
    Mat dilateElement = getStructuringElement( MORPH_RECT,Size(2*(2) + 1,2*(2) + 1));
    Mat imgsobel = sobelfilter(src);
    mass_road_tmp = MassOfRoad(src); //chu y day la mass_road o trong anh goc

    Mat bev_img = birdViewTranform(imgsobel, mass_road_tmp); //chu y day la mass road trong anh BirdView
    
    mass_road_BirdView.x = mass_road_tmp.x;
    mass_road_BirdView.y = mass_road_tmp.y;
    
    // fillLane(bev_img);
    // vector<Vec4i> lines;

    // HoughLinesP(bev_img, lines, 1, CV_PI / 180, 1, 0, 10);
    // for (size_t i = 0; i < lines.size(); i++)
    // {
    //     Vec4i l = lines[i]; //draw the line
    //     line(bev_img, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(255), 3, LINE_AA);

    //     // Point p1, p2;
    //     // p1=Point(l[0], l[1]);
    //     // p2=Point(l[2], l[3]);
    //     // //calculate angle in radian,  if you need it in degrees just do angle * 180 / PI
    //     // float angle = atan2(p1.y - p2.y, p1.x - p2.x)*(180/CV_PI);
    //     // cout << angle << endl;
    // }

    // dilate(bev_img,bev_img,dilateElement);
    // imshow("bird view",bev_img);
    return bev_img;
}

void DetectLane::morphOps(Mat &thresh, int* erode_kernel_size, int* dilate_kernel_size){
    Mat erodeElement = getStructuringElement( MORPH_RECT,Size(2*(*erode_kernel_size) + 1,2*( *erode_kernel_size) + 1));
    Mat dilateElement = getStructuringElement( MORPH_RECT,Size(2*(*dilate_kernel_size) + 1,2*( *dilate_kernel_size) + 1));

    erode(thresh,thresh,erodeElement);
    // erode(thresh,thresh,erodeElement);

    dilate(thresh,thresh,dilateElement);
    // dilate(thresh,thresh,dilateElement);
}

Mat DetectLane::sobelfilter(const Mat &src_rgb)
{
    Mat fill_lane_sobel;
    Mat img_gray_crop, imgHSV_crop;
    Mat canny_output, src_gray;
    int canny_thresh = 100;

    Rect roi;
    roi.x = 0;
    roi.y = 85;
    roi.width = src_rgb.size().width;
    roi.height = src_rgb.size().height - 85;
    Mat src_crop = src_rgb(roi);

    cvtColor(src_crop, imgHSV_crop, COLOR_BGR2HSV);

    // cvtColor(src_crop, src_gray, COLOR_BGR2GRAY);
    // Canny( src_gray, canny_output, canny_thresh, canny_thresh*2, 3 );
    // Mat canny_output_fullsize = canny_output_fullsize.zeros(Size(320, 240), src_rgb.type());
    // copyMakeBorder(canny_output, canny_output_fullsize, offset_y, 0, 0, 0, BORDER_CONSTANT, Scalar(0, 0, 0));
    // imshow("canny", canny_output);
    
    // inRange(imgHSV_crop, Scalar(minHSV[0], minHSV[1], minHSV[2]),Scalar(maxHSV[0], maxHSV[1], maxHSV[2]), img_gray_crop);
    inRange(imgHSV_crop, minHSV_lane, maxHSV_lane, img_gray_crop);
    morphOps(img_gray_crop, &erode_kernel_size, &dilate_kernel_size);

    Mat img_gray_fullSize = img_gray_fullSize.zeros(Size(320, 240), src_rgb.type());
    copyMakeBorder(img_gray_crop, img_gray_fullSize, offset_y, 0, 0, 0, BORDER_CONSTANT, Scalar(0, 0, 0));
    // imshow("after inrange",img_gray_fullSize);

    int ddepth = CV_8U;
    int scale = 1;
    int delta = 0;
    Mat grad_x, grad_y, abs_grad_x, abs_grad_y, grad, thresh_grad;

    Sobel(img_gray_fullSize, grad_x, ddepth, 1, 0, 3, scale, delta, BORDER_DEFAULT);
    convertScaleAbs(grad_x, abs_grad_x);
    Sobel(img_gray_fullSize, grad_y, ddepth, 0, 1, 3, scale, delta, BORDER_DEFAULT);
    convertScaleAbs(grad_y, abs_grad_y);

    addWeighted(abs_grad_x, alpha_addweight_sobfunc, abs_grad_y, beta_addweight_sobfunc, 0, grad);
    threshold(grad, thresh_grad, bin_threshold_sobfunc , 255, THRESH_BINARY);
    
    vector<Vec4i> lines;
    HoughLinesP(thresh_grad, lines, 1, CV_PI / 180, 1, 0, 7);
    Point line_approx_1 = null, line_approx_2 = null; 
    for (size_t i = 0; i < lines.size(); i++)
    {
        Vec4i l = lines[i]; //draw the line
        if (i == 0)
            line_approx_1 = Point(l[0], l[1]);
        else if (i == (lines.size() -1))
            line_approx_2 = Point(l[2], l[3]);

        line(thresh_grad, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(255), 3, LINE_AA);

        // Point p1, p2;
        // p1=Point(l[0], l[1]);
        // p2=Point(l[2], l[3]);
        // // calculate angle in radian,  if you need it in degrees just do angle * 180 / PI
        // float angle = atan2(p1.y - p2.y, p1.x - p2.x)*(180/CV_PI);
        // cout << angle << endl;
    }

    // Mat het_ten_dat = het_ten_dat.zeros(Size(320, 240), src_rgb.type());
    // line(het_ten_dat, line_approx_1, line_approx_2, Scalar(0, 0, 255), 4, LINE_8);
    // imshow("het ten", het_ten_dat);
#ifdef SHOW_WINDOW
    // imshow("after add", thresh_grad);
#endif
    // imshow("fill_lane_sobel", fill_lane_sobel);
    return thresh_grad;
}

Point DetectLane::MassOfRoad(const Mat &src_rgb)
{
    
    Mat src_HSV, road_thresh, road_thresh_inv;
    Point center = Point(0, 0);

    // int offset_y = 80;
    Rect roi;
    roi.x = 0;
    roi.y = offset_y;
    roi.width = src_rgb.size().width;
    roi.height = src_rgb.size().height - offset_y;
    Mat src_crop = src_rgb(roi);

    cvtColor(src_crop, src_HSV, COLOR_BGR2HSV);
    #ifdef CALIB
    inRange(src_HSV, Scalar(minHSV[0], minHSV[1], minHSV[2]), Scalar(maxHSV[0], maxHSV[1], maxHSV[2]), road_thresh);
    #else
    inRange(src_HSV, minHSV_massofroad, maxHSV_massofroad, road_thresh);
    #endif
    
    // bitwise_not(road_thresh, road_thresh_inv);

    Mat thresh_fullSize = thresh_fullSize.zeros(Size(320, 240), src_rgb.type());
    copyMakeBorder(road_thresh, thresh_fullSize, offset_y, 0, 0, 0, BORDER_CONSTANT, Scalar(0, 0, 0));
    /// Find contours
    vector<vector<Point> > contours; //Tao 1 vector 2 chieu voi moi phan tu la Point
    vector<Vec4i> hierarchy;
    findContours(thresh_fullSize, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0));

    Point mc_hsv = detectMassRoad(contours);

    #ifdef CALIB
    // circle(road_thresh, mc_hsv, 10, 0, -1, 8, 0);
    // imshow("massroad",road_thresh);
    
    
    Mat drawing = Mat::zeros(thresh_fullSize.size(), CV_8UC1);
    for (int i = 0; i < contours.size(); i++)
    {
        drawContours(drawing, contours, i, 255, 2, 8, hierarchy, 0, Point());
        circle(drawing, mc_hsv, 3, 255, -1, 8, 0);
    }
    imshow("road contour", drawing);
    #endif
    return mc_hsv;

}

Point DetectLane::detectMassRoad(vector<vector<Point> > &contours)
{
    vector<Point> max_contours;
    double max_area = 0;
    int max_index = 0;
    for (size_t i = 0; i < contours.size(); i++)
    {
        double area = fabs(contourArea(contours[i]));
        if (area > max_area)
        {
            max_area = area;
            max_index = i;
            max_contours = contours[max_index];
        }
    }
    Moments mu = moments(max_contours, false);
    return Point(mu.m10 / mu.m00, mu.m01 / mu.m00);
}

Mat DetectLane::birdViewTranform(const Mat &src, Point &mass_road)
{
    Point2f src_vertices[4];
    Mat dst(BIRDVIEW_HEIGHT, BIRDVIEW_WIDTH, CV_8UC3);

    int width = src.size().width;
    int height = src.size().height;

    src_vertices[0] = Point(0, offset_y);
    src_vertices[1] = Point(width, offset_y);
    src_vertices[2] = Point(width, height);
    src_vertices[3] = Point(0, height);

    // src_vertices[0] = Point(100, skyLine-10);
    // src_vertices[1] = Point(width-100, skyLine-10);
    // src_vertices[2] = Point(width, height-60);
    // src_vertices[3] = Point(0, height-60);

    Point2f dst_vertices[4];
    dst_vertices[0] = Point(0, 0);
    dst_vertices[1] = Point(BIRDVIEW_WIDTH, 0);
    dst_vertices[2] = Point(BIRDVIEW_WIDTH - 90, BIRDVIEW_HEIGHT);
    dst_vertices[3] = Point(90, BIRDVIEW_HEIGHT);

    Mat M = getPerspectiveTransform(src_vertices, dst_vertices);
    warpPerspective(src, dst, M, dst.size(), INTER_LINEAR, BORDER_CONSTANT);
    // tranform massroad
    vector<Point2f> dstPoints, srcPoints;
    srcPoints.push_back(Point2f(mass_road));
    perspectiveTransform(srcPoints, dstPoints, M);
    mass_road = Point(dstPoints.at(0).x, dstPoints.at(0).y );
    return dst;
}





/////////////////////////////////////////**  Everything About Lane **/////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
void DetectLane::fillLane(Mat &src)
{
    vector<Vec4i> lines;
    HoughLinesP(src, lines, 1, CV_PI / 180, 80);
    for (size_t i = 0; i < lines.size(); i++)
    {
        Vec4i l = lines[i]; //draw the line
        line(src, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(255), 3, LINE_AA);

        // Point p1, p2; //Tinh goc (angle) cua line
        // p1=Point(l[0], l[1]);
        // p2=Point(l[2], l[3]);
        // //calculate angle in radian,  if you need it in degrees just do angle * 180 / PI
        // float angle = atan2(p1.y - p2.y, p1.x - p2.x)*(180/CV_PI);
        // cout << angle << endl;
    }   
}

vector<Mat> DetectLane::splitLayer(const Mat &src, int dir)
{
    int rowN = src.rows;
    int colN = src.cols;
    std::vector<Mat> res;

    if (dir == VERTICAL)
    {
        for (int i = 0; i < rowN - slideThickness; i += slideThickness)
        {
            Mat tmp;
            Rect crop(0, i, colN, slideThickness);
            tmp = src(crop);
            res.push_back(tmp);
        }
    }
    else
    {
        for (int i = 0; i < colN - slideThickness; i += slideThickness)
        {
            Mat tmp;
            Rect crop(i, 0, slideThickness, rowN);
            tmp = src(crop);
            res.push_back(tmp);
        }
    }

    return res;
}

vector< vector<Point> > DetectLane::centerRoadSide(const vector<Mat> &src, int dir)
{
    vector< std::vector<Point> > res;
    int inputN = src.size();
    for (int i = 0; i < inputN; i++)
    {
        std::vector< std::vector<Point> > cnts;
        std::vector<Point> tmp;
        findContours(src[i], cnts, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, Point(0, 0));
        int cntsN = cnts.size();
        if (cntsN == 0)
        {
            res.push_back(tmp);
            continue;
        }
        for (int j = 0; j < cntsN; j++)
        {
            int area = contourArea(cnts[j], false);
            if (area > 3)
            {
                Moments M1 = moments(cnts[j], false);
                Point2f center1 = Point2f(static_cast<float>(M1.m10 / M1.m00), static_cast<float>(M1.m01 / M1.m00));
                if (dir == VERTICAL)
                {
                    center1.y = center1.y + slideThickness * i;
                }
                else
                {
                    center1.x = center1.x + slideThickness * i;
                }
                if (center1.x > 0 && center1.y > 0)
                {
                    tmp.push_back(center1);
                }
            }
        }
        res.push_back(tmp);
    }
    return res;
}

void DetectLane::detectLeftRight(const vector< vector<Point> > &points, Point &mass_road)
{
// Bird View: 320x240
    Mat test_left_rgt =  test_left_rgt.zeros(Size(240,320), CV_8UC3);
    float ind_lane1 = 0.0, ind_lane2 = 0.0;
    //** Bien khoang cach khi nhan duoc 2 lane nhung thuc te co chi co 1 lane bi tach lam 2 **//
    int dy_defect_1, dy_defect_2, dy_min;
    int dx_defect_1, dx_defect_2, dx_min;
    //** Bien tam cua 2 lane1 va lane2
    Point center_lane1 = null, center_lane2 = null;

    static vector<Point> lane1, lane2;
    int lane1_predict, lane2_predict;
    lane1.clear();
    lane2.clear();
    leftLane.clear();
    rightLane.clear();
//** Tinh toan de xap xi 2 lane => N diem trong lane1 va lane2 (2 vector<Points>) **//
    for (int i = 0; i < BIRDVIEW_HEIGHT / slideThickness; i++) //i< 320/10 = 32
    {
        leftLane.push_back(null);
        rightLane.push_back(null);
    }

    int pointMap[points.size()][20];
    int prePoint[points.size()][20];
    int postPoint[points.size()][20];
    int dis = 10;
    int max1 = -1, max2 = -1;
    Point2i posMax1, posMax2;

    Rect top_rect(Point(0, 0), Point(BIRDVIEW_WIDTH, 1));
    Rect right_rect(Point(BIRDVIEW_WIDTH - 50, 0), Point(BIRDVIEW_WIDTH, BIRDVIEW_HEIGHT));

    memset(pointMap, 0, sizeof pointMap);

    for (int i = 0; i < points.size(); i++)
    {
        for (int j = 0; j < points[i].size(); j++)
        {
            pointMap[i][j] = 1;
            prePoint[i][j] = -1;
            postPoint[i][j] = -1;
        }
    }

    for (int i = points.size() - 2; i >= 0; i--)
    {
        for (int j = 0; j < points[i].size(); j++)
        {
            int err = 320;
            for (int k = 0; k < points[i + 1].size(); k++)
            {
                if (abs(points[i + 1][k].x - points[i][j].x) < dis &&
                    abs(points[i + 1][k].x - points[i][j].x) < err)
                {
                    err = abs(points[i + 1][k].x - points[i][j].x);
                    pointMap[i][j] = pointMap[i + 1][k] + 1;
                    prePoint[i][j] = k;
                    postPoint[i + 1][k] = j;
                }
            }
        }
    }
    // push_back lane point
    for (int i = 0; i < points.size(); i++)
    {
        for (int j = 0; j < points[i].size(); j++)
        {
            if (pointMap[i][j] > max1 && postPoint[i][j] == -1 )//& ~point_in_rect(top_rect, points[i].at(j)))
            // & ~point_in_rect( right_rect, points[i].at(j) ))
            {
                max1 = pointMap[i][j];
                posMax1 = Point2i(i, j);
            }
        }
    }
    for (int i = 0; i < points.size(); i++)
    {
        for (int j = 0; j < points[i].size(); j++)
        {
            if (pointMap[i][j] > max2 && (i != posMax1.x || j != posMax1.y) && postPoint[i][j] == -1 )
            //& ~point_in_rect(top_rect, points[i].at(j)))
            // & ~point_in_rect( right_rect, points[i].at(j) ) )
            {

                max2 = pointMap[i][j];
                posMax2 = Point2i(i, j);
            }
        }
    }

    if (max1 == -1)
        return;
    while (max1 >= 1)
    {
        lane1.push_back(points[posMax1.x][posMax1.y]);
        if (max1 == 1)
            break;

        posMax1.y = prePoint[posMax1.x][posMax1.y];
        posMax1.x += 1;

        max1--;
    }

    while (max2 >= 1)
    {
        lane2.push_back(points[posMax2.x][posMax2.y]);
        if (max2 == 1)
            break;

        posMax2.y = prePoint[posMax2.x][posMax2.y];
        posMax2.x += 1;

        max2--;
    }

    for (int i = 1; i < lane1.size(); i++)
    {
        if (lane1[i] != null)
        {
            circle(test_left_rgt, lane1[i], 1, Scalar(0, 0, 255), 2, 8, 0); //mau do lane trai
            // putText(test_left_rgt, Utility().intToString(i), lane1[i], FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 0, 0), 1);
            ind_lane1 = ind_lane1 + 1;
        }
    }

    for (int i = 1; i < lane2.size(); i++)
    {
        if (lane2[i] != null)
        {
            circle(test_left_rgt, lane2[i], 1, Scalar(255, 0, 0), 2, 8, 0); //mau xanh duong lane phai
            // putText(test_left_rgt, Utility().intToString(i), lane2[i], FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 0, 0), 1);
            ind_lane2 = ind_lane2 + 1;
        }
    }
//** Tinh toan de xap xi 2 lane => N diem trong lane1 va lane2 (2 vector<Points>) **//

//** Xac dinh tam cua lane1 va lane2 **//
    int tmp_ind_1 = ind_lane1/2;
    int tmp_ind_2 = ind_lane2/2;
    if (tmp_ind_1 < 3){ //loai bo do co it hon 6 diem (co the do nhieu)
        center_lane1 = null;
    }
    else{
        center_lane1 = Point(lane1[tmp_ind_1].x , lane1[tmp_ind_1].y);
    }

    if (tmp_ind_2 < 3){ //loai bo do co it hon 6 diem (co the do nhieu)
        center_lane2 = null;
    }
    else{
        center_lane2 = Point(lane2[tmp_ind_2].x , lane2[tmp_ind_2].y);
    }

    // if ( (lane1[0] != null) && (lane1[ind_lane1] != null) )
    //     line(test_left_rgt, lane1[0], lane1[ind_lane1], Scalar(27, 50, 255), 3, LINE_8);
    // if ( (lane2[0] != null) && (lane2[ind_lane2] != null) )
    //     line(test_left_rgt, lane2[0], lane2[ind_lane2], Scalar(27, 50, 255), 3, LINE_8);
//** Xac dinh do lech khoang cach giua 2 lane **//
	if ( ((int) ind_lane1 != 0) && ( (int)ind_lane2 != 0 ) )
	{
		dy_defect_1 = abs(lane1[(int)ind_lane1].y    - lane2[0].y);
		dy_defect_2 = abs(lane1[0].y            - lane2[(int)ind_lane2].y);
		// dx_defect_1 = abs(lane1[ind_lane1].x    - lane2[0].x);
		// dx_defect_2 = abs(lane1[0].x            - lane2[ind_lane2].x);
		dy_min = (dy_defect_1 < dy_defect_2 ) ? dy_defect_1 : dy_defect_2;
		if (dy_min == dy_defect_1)
			dx_min = abs(lane1[(int)ind_lane1].x    - lane2[0].x);
		else 
			dx_min = abs(lane1[0].x            - lane2[(int)ind_lane2].x);
		// dx_min = (dy_defect_1 < dy_defect_2 ) ? dx_defect_1 : dx_defect_2;
		// dx_min = (dx_defect_1 < dx_defect_2 ) ? dx_defect_1 : dx_defect_2;
	}

//** Xac dinh goc lech cua duong thang xap xi 2 lane **//

#ifdef SHOW_WINDOW
    // imshow("test_left_rgt", test_left_rgt);
#endif

    //** Detect left or right lane start here **//
    static double var1 = 0, var2 = 0;
    int16_t lost_lane1 = 0, lost_lane2 = 0;
    lost_lane1 = Lost_lane(lane1);
    lost_lane2 = Lost_lane(lane2);
    if ( (dy_min < 60) && (dx_min < 17) ){ //chi co 1 lane nhung bi dut doan thanh 2 lane
        lane1_predict = recognize_left_right(lane1, mass_road, var1);
        if (lane1_predict == LEFT_LANE)
            lane2_predict = LEFT_LANE;
        else if (lane1_predict == RIGHT_LANE)
            lane2_predict = RIGHT_LANE;
    }

    else
    {
        if (lost_lane1 == -1){ //mat 1 lane
        // if (ind_lane1 <5){
            lane2_predict = recognize_left_right(lane2, mass_road, var2);
            // lane2_predict = Detect_one_lane(lane2, mass_road_src, var2, angle_1, angle_2);
        }
        if (lost_lane2 == -1){ //mat 1 lane
        // if (ind_lane2 < 5){
            lane1_predict = recognize_left_right(lane1, mass_road, var1);
            // lane1_predict = Detect_one_lane(lane1, mass_road_src, var1, angle_1, angle_2);
        }
        if ( (lost_lane1 != -1) && (lost_lane2 != -1) ){ //co ca 2 lane
        // if ( (ind_lane2 > 5) && (ind_lane1 > 5) ){
            lane1_predict = recognize_left_right(lane1, mass_road, var1);
            lane2_predict = recognize_left_right(lane2, mass_road, var2);
        }
    }

    
    variance = (var1 > var2 ) ? var1 : var2;
    // cout << "variance: " << variance << endl;
    if (variance > 30)
        turning_ahead = true;
    else
        turning_ahead = false;

    if (lane1_predict == LEFT_LANE)
    {
        for (int i = 0; i < lane1.size(); i++)
        {
            leftLane[floor(lane1[i].y / slideThickness)] = lane1[i];
            // leftLane[i] = lane1[i];
        }
    }
    else if (lane1_predict == RIGHT_LANE)
    {
        for (int i = 0; i < lane1.size(); i++)
        {
            rightLane[floor(lane1[i].y / slideThickness)] = lane1[i];
            // rightLane[i] = lane1[i];
        }
    }
    else
    {
        return;
    }

    // if (max2 == -1)
    //     return;

    if (lane2_predict == LEFT_LANE)
    {
        for (int i = 0; i < lane2.size(); i++)
        {
            leftLane[floor(lane2[i].y / slideThickness)] = lane2[i];
            // leftLane[i] = lane2[i];
        }
    }
    else if (lane2_predict == RIGHT_LANE)
    {
        for (int i = 0; i < lane2.size(); i++)
        {
            rightLane[floor(lane2[i].y / slideThickness)] = lane2[i];
        }
    }
    else
    {
        return;
    }
    //** Calculate center of leftLane & rightLane start here **//
    if ( (lane1_predict == RIGHT_LANE) && (lane2_predict == LEFT_LANE) )
    {
        if (center_lane1 == null){ //Chi co center lane2, mat center lane1
            center_lft_lane = center_lane2;
            center_road_lane = Point( center_lft_lane.x + half_laneWidth, center_lft_lane.y );
        }
        else if (center_lane2 == null){ //chi co center lane1, mat center lane2
            center_rgt_lane = center_lane1;
            center_road_lane = Point( center_rgt_lane.x - half_laneWidth, center_rgt_lane.y );
        }
        else if ( (center_lane1 != null) && (center_lane2 != null)){ //co ca hai center
            center_lft_lane = center_lane2;
            center_rgt_lane = center_lane1;
            center_road_lane = Point( (center_rgt_lane.x + center_lft_lane.x)/2, (center_rgt_lane.y + center_lft_lane.y)/2 );
        }
    }

    else if ( (lane1_predict == LEFT_LANE) && (lane2_predict == RIGHT_LANE) )
    {
        if (center_lane1 == null){
            center_rgt_lane = center_lane2;
            center_road_lane = Point( center_rgt_lane.x - half_laneWidth, center_rgt_lane.y );
        }
        else if (center_lane2 == null){
            center_lft_lane = center_lane1;
            center_road_lane = Point( center_lft_lane.x + half_laneWidth, center_lft_lane.y );
        }
        else if ( (center_lane1 != null) && (center_lane2 != null)){
            center_rgt_lane = center_lane2;
            center_lft_lane = center_lane1;
            center_road_lane = Point( (center_rgt_lane.x + center_lft_lane.x)/2, (center_rgt_lane.y + center_lft_lane.y)/2 );
        }
    }

    else if ( (lane1_predict == LEFT_LANE) && (lane2_predict == LEFT_LANE) ){ //chi co 1 lane trai
        // center_lft_lane = (center_lane1+center_lane2)/2;

        // if (center_lane1.y >  center_lane2.y) //lay diem nam duoi
        //     center_lft_lane = center_lane1;
        // else
        //     center_lft_lane = center_lane2;

        // int dist_1 = abs( lane1[0].y - lane2[ind_lane2].y);
        // int dist_2 = abs( lane1[ind_lane1].y - lane2[0].y);
        // if ( dist_1 < dist_2)
        //     center_lft_lane = (lane1[0] + lane2[ind_lane2])/2;
        // else if (dist_1 > dist_2)
        //     center_lft_lane = (lane1[ind_lane1] + lane2[0])/2;

        if ((int) ind_lane1 > (int) ind_lane2)
            center_lft_lane = center_lane1;
        else if ((int) ind_lane1 <= (int) ind_lane2)
            center_lft_lane = center_lane2;

        center_road_lane = Point( center_lft_lane.x + half_laneWidth, center_lft_lane.y );
    }

    else if ( (lane1_predict == RIGHT_LANE) && (lane2_predict == RIGHT_LANE) ){ //chi co 1 lane phai
        // center_rgt_lane = (center_lane1+center_lane2)/2;

        // if (center_lane1.y >  center_lane2.y) //lay diem nam duoi
        //     center_rgt_lane = center_lane1;
        // else
        //     center_rgt_lane = center_lane2;

        // int dist_1 = abs( lane1[0].y - lane2[ind_lane2].y);
        // int dist_2 = abs( lane1[ind_lane1].y - lane2[0].y);
        // if ( dist_1 < dist_2)
        //     center_rgt_lane = (lane1[0] + lane2[ind_lane2])/2;
        // else if (dist_1 > dist_2)
        //     center_rgt_lane = (lane1[ind_lane1] + lane2[0])/2;

        if ((int)ind_lane1 > (int)ind_lane2)
            center_rgt_lane = center_lane1;
        else if ((int)ind_lane1 <= (int)ind_lane2)
            center_rgt_lane = center_lane2;

        center_road_lane = Point( center_rgt_lane.x - half_laneWidth, center_rgt_lane.y );
    }

    else return;
    //** Calculate center of leftLane & rightLane end here **//
}

int DetectLane::recognize_left_right(vector<Point> &lanex, Point &mass_road, double& var)
{
    vector<Point> subLane;
    static int cnt;
    for (size_t i = 0; i < lanex.size(); i++)
    {
        if (lanex.at(i) != null)
        {
            subLane.push_back(lanex.at(i));
        }
    }
    if (subLane.size() < 2)
        return -1;

    Vec4f line;
    fitLine(subLane, line, DIST_L1, 0, 0.01, 0.01);
    // caculate variance
    for (size_t i = 0; i < subLane.size(); i++)
    {
        double y_point = (double)subLane.at(i).y;
        double x_point = (double)subLane.at(i).x;
        double x_ref = line[0] / line[1] * (y_point - line[3]) + line[2];
        var += pow(abs(x_ref - x_point), 2.0);
    }
    var = sqrt(var);
    float y_mass = (float)(mass_road.y);
    float x_mass = (float)(mass_road.x);
    // x_ref = Vx/Vy * (y_point - yo) + xo
    float x_line = line[0] / line[1] * (y_mass - line[3]) + line[2];
    // true is left lane
    return (x_line <= x_mass) ? LEFT_LANE : RIGHT_LANE;
}

int16_t DetectLane::Detect_one_lane( vector<Point> &lanex, Point &mass_road, double& var, float ang_1, float ang_2)
{
    vector<Point> subLane;
    static int cnt;
    for (size_t i = 0; i < lanex.size(); i++)
    {
        if (lanex.at(i) != null)
        {
            subLane.push_back(lanex.at(i));
        }
    }
    if (subLane.size() < 2)
        return -1;

    Vec4f line;
    fitLine(subLane, line, DIST_L1, 0, 0.01, 0.01);

    for (size_t i = 0; i < subLane.size(); i++)
    {
        double y_point = (double)subLane.at(i).y;
        double x_point = (double)subLane.at(i).x;
        double x_ref = line[0] / line[1] * (y_point - line[3]) + line[2];
        var += pow(abs(x_ref - x_point), 2.0);
    }
    var = sqrt(var);

    if ( (ang_1 < -105) ){ //mat lane trai, lane con lai la lane phai
        return RIGHT_LANE;
    }
    else if ( (ang_1 > -70) ){ //mat lane phai, lane con lai la lane trai
        return LEFT_LANE;
    }
    else
		return 0;
}

int16_t DetectLane::Lost_lane(const vector<Point> &points) //neu tra ve -1 tuc la mat lane, neu tra ve 0 tuc la co lane
{
    int i = 0, idx_null = 0;
    while(i < points.size())
    {
        if (points[i] != null)
            break;
        else
            idx_null++;
        i++;
    }
    if (idx_null == points.size())
        return -1;
    else
        return 0;
}

bool DetectLane::point_in_rect(Rect rect_win, Point p)
{
    int x_min = rect_win.tl().x;
    int y_min = rect_win.tl().y;
    int x_max = rect_win.br().x;
    int y_max = rect_win.br().y;
    bool in_rect = (p.x >= x_min);
    in_rect = in_rect & (p.x <= x_max);
    in_rect = in_rect & (p.y >= y_min);
    in_rect = in_rect & (p.y <= y_max);
    return in_rect;
}
