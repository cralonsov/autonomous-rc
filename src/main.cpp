#include <iostream>
#include <vector>
#include <cmath>
#include <numeric>
#include <linux/i2c-dev.h>
#include <linux/types.h>
#include <chrono>
#include <thread>
#include <string>

#include "PCA9685.h"
#include "PID.h"

#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>


struct Lane
{
    double m, b;
};


void window_management(cv::Mat& window, const std::string& windowName, bool& showHelp, bool& fullScreen, const char& key);

cv::Mat region_of_interest(const cv::Mat& src, const std::vector<cv::Point>& v);

void get_lanes(const std::vector<cv::Vec4i> lines, Lane& right, Lane& left, std::vector<double>& weights_r, std::vector<double>& weights_l, const int& srcWidth, const int& srcHeight);
void draw_lane(cv::Mat& line_image, const Lane& lane, int& x, const int& srcWidth, const int& srcHeight, const cv::Scalar& color);
void intersection_point(cv::Point& inter, const int& x0_r, const int& x1_r, const int& y0_r, const int& y1_r, const int& x0_l, const int& x1_l, const int& y0_l, const int& y1_l);

int main(int argc, const char** argv)
{
    // Motor related parameters
    PCA9685 pwm(1, 0x40);
    pwm.setPwmFreq(65);
    
    PID pid = PID(0.1, LEFT_MAX, RIGHT_MAX, 0.3, 0, 0);

    const uint8_t motor = 0;
    const uint8_t steering = 1;
    double speed = 382.0;

    // Canny's related parameters
    const double lowThreshold = 100.0;
    const double highThreshold = 150.0;
    const int minLineLength = 50;
    const int maxLineGap = 10;
    int ratio = 3;
    int kernel_size = 3;
    int houghVote = 200;
    cv::Vec4f r_lane, l_lane;
    int x_r, x_l;

    bool showHelp = true;
    bool fullScreen = false;
    const std::string windowName = "Camera Output";

    const std::string gst =  "nvcamerasrc ! video/x-raw(memory:NVMM), width=(int)1024, height=(int)768, \
                            format=(string)I420, framerate=(fraction)60/1 ! \
                            nvvidconv flip-method=0 ! video/x-raw, format=(string)BGRx ! \
                            videoconvert ! video/x-raw, format=(string)BGR ! \
                            appsink";

    cv::VideoCapture cap(gst);
    //cap.open("../media/sample-videos/video_20180531_201943.avi");

    if(!cap.isOpened())
    {
        std::cout << "Failed to open camera." << std::endl;
        return -1;
    }

    cv::Mat src, edges, dst, masked_image;
    cv::namedWindow(windowName, 1);

    double dWidth = cap.get(CV_CAP_PROP_FRAME_WIDTH); 
    double dHeight = cap.get(CV_CAP_PROP_FRAME_HEIGHT); 
    cv::Size frameSize(dWidth, dHeight);

    while(true)
    {
        cap >> src;  // Get a new Frame from the camera
        
        if(src.empty())
            break;
        
        int srcWidth = src.cols;
        int srcHeight = src.rows;

        cv::cvtColor(src, edges, cv::COLOR_BGR2GRAY);
        cv::GaussianBlur(edges, edges, cv::Size(kernel_size, kernel_size), 1.5, 1.5);
        cv::Canny(edges, edges, lowThreshold, highThreshold, kernel_size);

        dst = src.clone();
        
        char key = (char)cv::waitKey(1);

        cv::Mat mask = cv::Mat::zeros(edges.rows, edges.cols, CV_32F);

        std::vector<cv::Point> vertices{cv::Point(srcWidth, srcHeight), \
                                        cv::Point(0, srcHeight), \
                                        cv::Point(0, int(2*srcHeight/3)), \
                                        cv::Point(srcWidth, int(2*srcHeight/3))};

        masked_image = region_of_interest(edges, vertices);


        //Probabilistic Hough Line Transform
        std::vector<cv::Vec4i> lines;
        
        cv::HoughLinesP(masked_image, lines, 1.0f, (float) (CV_PI / 180.0f), minLineLength, maxLineGap);
        cv::Mat line_image = cv::Mat::zeros(srcHeight, srcWidth, src.type());
        
        //std::vector<cv::Vec2i> r, l;
        std::vector<double> weights_r, weights_l;
        Lane right, left;
        
        get_lanes(lines, right, left, weights_r, weights_l, srcWidth, srcHeight);
        
        draw_lane(line_image, right, x_r, srcWidth, srcHeight, cv::Scalar(255, 0, 0));
        draw_lane(line_image, left, x_l, srcWidth, srcHeight, cv::Scalar(255, 0, 0));
        //draw_lane(line_image, l, l_lane, x0_l, x1_l, y0_l, y1_l, srcWidth, srcHeight, cv::Scalar(255, 0, 0));
        
        // Measure where is the center of the lane and where you are placed
        double center = srcWidth/2;
        double inter = x_l + (x_r - x_l)/2.0;
        double pos = inter - center + DIR_REST;      
        
        cv::line(line_image, cv::Point(inter, srcHeight), cv::Point(inter, srcHeight-50), cv::Scalar(0, 0, 255), 3, cv::LINE_AA);
        cv::line(line_image, cv::Point(center, srcHeight), cv::Point(center, srcHeight-50), cv::Scalar(0, 255, 0), 3, cv::LINE_AA);

        double alpha = 0.8; 
        double beta = 0.4;
        
        cv::addWeighted(src, alpha, line_image, beta, 0.0, dst);
        
        if (key == 27)
            break;
            
        double inc = pid.calculate(395, pos);
        pwm.setPwm(steering, inc);
        
        
        if(pos < 395) //Turn left
        {
            //pwm.setPwm(steering, RIGHT_MAX);
            //std::this_thread::sleep_for(std::chrono::microseconds(1000000));
            std::string strPos = "Turn left: " + std::to_string(pos) + " - " + std::to_string(inc);
            cv::putText(dst, strPos, cv::Point(10,40), cv::FONT_HERSHEY_DUPLEX, 0.5, CV_RGB(240,240,240), 1);
        }
        else //Turn right
        {
            std::string strPos = "Turn right: " + std::to_string(pos) + " - " + std::to_string(inc);
            cv::putText(dst, strPos, cv::Point(10,40), cv::FONT_HERSHEY_DUPLEX, 0.5, CV_RGB(240,240,240), 1);
            //pwm.setPwm(steering, LEFT_MAX);
            //std::this_thread::sleep_for(std::chrono::microseconds(1000000));
        }

        window_management(dst, windowName, showHelp, fullScreen, key);
        
        pwm.setPwm(motor, speed);

        cv::namedWindow("Lines", CV_WINDOW_NORMAL);
        cv::resizeWindow("Lines", int(srcWidth/2), int(srcHeight/2));
        cv::moveWindow("Lines", srcWidth+80, 0);

        cv::namedWindow("Canny Edge", CV_WINDOW_NORMAL);
        cv::resizeWindow("Canny Edge", int(srcWidth/2),int(srcHeight/2));
        cv::moveWindow("Canny Edge", srcWidth+80, int(srcHeight/2)+80);

        cv::imshow(windowName, dst);
        std::string strText = "Left m: " + std::to_string(left.m) + ", Right m: " + std::to_string(right.m);
        cv::putText(line_image, strText, cv::Point(10,40), cv::FONT_HERSHEY_DUPLEX, 0.5, CV_RGB(240,240,240), 1);
        std::cout << strText << std::endl;
        strText = "Left b: " + std::to_string(left.b) + ", Right b: " + std::to_string(right.b);
        cv::putText(line_image, strText, cv::Point(10,80), cv::FONT_HERSHEY_DUPLEX, 0.5, CV_RGB(240,240,240), 1);
        std::cout << strText << std::endl;
        cv::imshow("Lines", line_image);
        cv::imshow("Canny Edge", masked_image);
    }
    
    pwm.setAllPwm(0, 0);

    return 0;
}


void window_management(cv::Mat& window, const std::string& windowName, bool& showHelp, bool& fullScreen, const char& key)
{
    const std::string helpText = "'Esc' to Quit, 'H' to Toggle Help, 'F' to Toggle Fullscreen";

    if (showHelp == true)
        cv::putText(window, helpText, cv::Point(10,20), cv::FONT_HERSHEY_DUPLEX, 0.5, CV_RGB(240,240,240), 1);

    if (fullScreen == true)
            cv::setWindowProperty(windowName, CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);

        else
            cv::setWindowProperty(windowName, CV_WND_PROP_FULLSCREEN, CV_WINDOW_NORMAL);

    if (key == 'H' || key == 'h')
        showHelp = !showHelp;

    else if (key == 'F' || key == 'f')
        fullScreen = !fullScreen;

}

cv::Mat region_of_interest(const cv::Mat& src, const std::vector<cv::Point>& v)
{
    cv::Mat mask = cv::Mat::zeros(src.rows, src.cols, src.type());

    std::vector<std::vector<cv::Point> > fillContAll;
    
    // Fill the single contour
    cv::Scalar color;
    if ( src.channels() > 2 )
        color = cv::Scalar(255, 255, 255);
    else
        color = cv::Scalar(255);

    fillContAll.push_back(v);
    cv::fillPoly(mask, fillContAll, cv::Scalar(255, 255, 255));

    cv::Mat masked_image;
    cv::bitwise_and(src, mask, masked_image);

    return masked_image;
}


void draw_lane(cv::Mat& line_image, const Lane& lane, int& x, const int& srcWidth, const int& srcHeight, const cv::Scalar& color)
{    
    int y0 = srcHeight * 2/3;
    int x0 = (y0 - lane.b) / lane.m * (lane.m != -1.0) + ((lane.m == -1.0) * lane.b);
    int y1 = srcHeight;
    int x1 = (y1 - lane.b) / lane.m * (lane.m != -1.0) + ((lane.m == -1.0) * lane.b);
    
    x = x0;
    
    cv::line(line_image, cv::Point(x0, y0), cv::Point(x1, y1), color, 3, cv::LINE_AA);
}


void get_lanes(const std::vector<cv::Vec4i> lines, Lane& right, Lane& left, std::vector<double>& weights_r, std::vector<double>& weights_l, const int& srcWidth, const int& srcHeight)
{    
    //std::vector<cv::Vec2i> r, l;
    std::vector<double> r_m, r_b, l_m, l_b;
    Lane lane;
    
    for (size_t i = 0; i < lines.size(); ++i)
    {
        cv::Point p1 = cv::Point(lines[i][0], lines[i][1]);
        cv::Point p2 = cv::Point(lines[i][2], lines[i][3]);
        double length;
        
        if(p2.x - p1.x != 0)
        {
            lane.m = (p2.y - p1.y) / (double)(p2.x - p1.x);
            lane.b = p1.y - lane.m * p1.x;
            length = sqrt(pow(p2.x-p1.x, 2.0) + pow(p2.y-p1.y, 2.0));
        }
        
        // Remember coordinates in OpenCV are different (The y is upside-down)
        if(lane.m < -0.2 && p1.x < srcWidth/2 && p2.x < srcWidth/2)
        {
            l_b.push_back(lane.b); l_m.push_back(lane.m);
            weights_l.push_back(length);
        }
        else if(lane.m > 0.2 && p1.x > srcWidth/2 && p2.x > srcWidth/2)
        {
            r_b.push_back(lane.b); r_m.push_back(lane.m);
            weights_r.push_back(length);
        }
    }
    
    if(r_m.size() != 0)
    {
        right.m = std::inner_product(weights_r.begin(), weights_r.end(), r_m.begin(), 0.0) / std::accumulate(weights_r.begin(), weights_r.end(), 0.0);
        right.b = std::inner_product(weights_r.begin(), weights_r.end(), r_b.begin(), 0.0) / std::accumulate(weights_r.begin(), weights_r.end(), 0.0);
    }
    else
    {
        right.m = -1.0;
        right.b = srcWidth;
    }
    
    if(l_m.size() != 0)
    {
        left.m = std::inner_product(weights_l.begin(), weights_l.end(), l_m.begin(), 0.0) / std::accumulate(weights_l.begin(), weights_l.end(), 0.0);
        left.b = std::inner_product(weights_l.begin(), weights_l.end(), l_b.begin(), 0.0) / std::accumulate(weights_l.begin(), weights_l.end(), 0.0);
    }
    else
    {
        left.m = -1.0;
        left.b = 0.0;
    }
}

// To get the intersection point between two lines (They have to have the intersection point visible)
void intersection_point(cv::Point& inter, const int& x0_r, const int& x1_r, const int& y0_r, const int& y1_r, const int& x0_l, const int& x1_l, const int& y0_l, const int& y1_l)
{
    float s1_x, s1_y, s2_x, s2_y;
    s1_x = x1_r - x0_r;     s1_y = y1_r - y0_r;
    s2_x = x1_l - x0_l;     s2_y = y1_l - y0_l;

    float s, t;
    s = (-s1_y * (x0_r - x0_l) + s1_x * (y0_r - y0_l)) / (-s2_x * s1_y + s1_x * s2_y);
    t = ( s2_x * (y0_r - y0_l) - s2_y * (x0_r - x0_l)) / (-s2_x * s1_y + s1_x * s2_y);

    if(s >= 0 && s <= 1 && t >= 0 && t <= 1)
    {
        inter.x = x0_r + (t * s1_x);
        inter.y = y0_r + (t * s1_y);
    }
}

