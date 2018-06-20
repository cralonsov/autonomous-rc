#include "opencv2/objdetect.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <iostream>
#include <stdio.h>
#include <string>

using namespace cv;

void detectAndDisplay(Mat frame);

std::string peopleCascadeName, faceCascadeName;
cv::CascadeClassifier peopleCascade;
cv::CascadeClassifier faceCascade;
std::string windowName = "Capture - People Detection";
const int WIDTH = 1920, HEIGHT=1080;

int main(int argc, const char** argv)
{
    peopleCascadeName = "./haarcascade_fullbody.xml";
    faceCascadeName = "./haarcascade_frontalface_alt.xml";
    
    cv::VideoCapture capture;
    cv::Mat frame;
    
    // Load the cascades
    if(!peopleCascade.load(peopleCascadeName))
    { 
        std::cout << "Error loading people cascade" << std::endl;
        return -1;
    }

    if(!faceCascade.load(faceCascadeName))
    { 
        std::cout << "Error loading face cascade" << std::endl;
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
        std::string srcName = "./image.jpg";
        std::string dstName = "./image_modified.png";

        frame = cv::imread(srcName, CV_LOAD_IMAGE_COLOR);

        // Check for invalid input
        if(! frame.data) 
        {
            std::cout << "Could not open or find the image" << std::endl;
            return -1;
        }

        detectAndDisplay(frame);

        imwrite(dstName, frame);

        waitKey(0);
    }

    return 0;
}
void detectAndDisplay(cv::Mat frame)
{
    std::vector<cv::Rect> people;
    cv::Mat frame_gray;
    cv::cvtColor(frame, frame_gray, COLOR_BGR2GRAY);
    cv::equalizeHist(frame_gray, frame_gray);
    
    // Detect people
    peopleCascade.detectMultiScale(frame_gray, people, 1.1, 3, 0|CASCADE_SCALE_IMAGE, cv::Size(200, 200));
    for(size_t i = 0; i < people.size(); i++)
    {
        cv::putText(frame, "Person", Point(people[i].x, people[i].y-10), cv::FONT_HERSHEY_DUPLEX, 2, CV_RGB(255,0,255), 3, cv::LINE_8);
        cv::rectangle(frame, Point(people[i].x, people[i].y), cv::Point(people[i].x + people[i].width, people[i].y + people[i].height), cv::Scalar(255, 0, 255), 10, 8, 0);
        cv::Mat peopleROI = frame_gray(people[i]);
        std::vector<Rect> faces;

        // In each person, detect faces
        faceCascade.detectMultiScale(peopleROI, faces, 1.1, 3, 0 |CASCADE_SCALE_IMAGE, cv::Size(30, 30));
        for(size_t j = 0; j < faces.size(); j++)
        {
            cv::Point face_center(people[i].x + faces[j].x + faces[j].width/2, people[i].y + faces[j].y + faces[j].height/2);
            int radius = cvRound((faces[j].width + faces[j].height)*0.25);
            cv::circle(frame, face_center, radius, cv::Scalar(0, 255, 0), 8, 8, 0);
        }
    }
    // Show what you got
    cv::namedWindow(windowName, CV_WINDOW_NORMAL);
    cv::resizeWindow(windowName, WIDTH, HEIGHT);
    cv::imshow(windowName, frame);
}