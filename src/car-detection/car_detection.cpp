#include "opencv2/objdetect.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <iostream>
#include <stdio.h>
#include <string>

using namespace cv;

void detectAndDisplay(Mat frame);

std::string carCascadeName;
cv::CascadeClassifier carCascade;

std::string windowName = "Capture - Car Detection";
const int WIDTH = 1920, HEIGHT=1080;

int main(int argc, const char** argv)
{
    carCascadeName = "./haarcascade_car.xml";
    
    
    cv::VideoCapture capture;
    cv::Mat frame;
    
    // Load the cascades
    if(!carCascade.load(carCascadeName))
    { 
        std::cout << "Error loading car cascade" << std::endl;
        return -1;
    }
        
    // Read the video stream
    if(argc > 1 && argv[1] == std::string("webcam"))
    {
        capture.open(0);

        if(!capture.isOpened())
        { 
            std::cout << "Error opening video capture" << std::endl;
            return -1;
        }

        while(capture.read(frame))
        {
            if(frame.empty())
            {
                std::cout << "No captured frame" << std::endl;
                break;
            }

            // Apply the classifier to the frame
            detectAndDisplay(frame);
            
            if(cv::waitKey(10) == 27)
                break;
        }
    }

    else
    {
        for(int i = 1; i < 5; i++)
        {
            std::string srcName = "./image" + std::to_string(i) + ".jpg";
            std::string dstName = "./image_modified" + std::to_string(i) + ".png";

            frame = cv::imread(srcName, CV_LOAD_IMAGE_COLOR);

            // Check for invalid input
            if(! frame.data) 
            {
                std::cout << "Could not open or find the image" << std::endl;
                return -1;
            }

            detectAndDisplay(frame);

            imwrite(dstName, frame);
        }

        waitKey(0);
    }

    return 0;
}
void detectAndDisplay(cv::Mat frame)
{
    std::vector<cv::Rect> car;
    cv::Mat frame_gray;
    cv::cvtColor(frame, frame_gray, COLOR_BGR2GRAY);
    cv::equalizeHist(frame_gray, frame_gray);
    
    // Detect car
    carCascade.detectMultiScale(frame_gray, car, 1.1, 3, 0|CASCADE_SCALE_IMAGE, cv::Size(80, 80));
    for(size_t i = 0; i < car.size(); i++)
    {
        cv::putText(frame, "Car", Point(car[i].x, car[i].y-10), cv::FONT_HERSHEY_DUPLEX, 2, CV_RGB(0,0,255), 3, cv::LINE_8);
        cv::rectangle(frame, Point(car[i].x, car[i].y), cv::Point(car[i].x + car[i].width, car[i].y + car[i].height), CV_RGB(0,0,255), 10, 8, 0);
        cv::Mat carROI = frame_gray(car[i]);
    }
    // Show what you got
    cv::namedWindow(windowName, CV_WINDOW_NORMAL);
    cv::resizeWindow(windowName, WIDTH, HEIGHT);
    cv::imshow(windowName, frame);
}