#include <iostream>
#include <vector>

#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>

void window_management(cv::Mat& window, const std::string& windowName, bool& showHelp, bool& fullScreen, const char& key);

cv::Mat region_of_interest(const cv::Mat& src, const std::vector<cv::Point>& v);

int main(int argc, const char** argv)
{
	// Canny's related parameters
	const double lowThreshold = 60.0;
	const double highThreshold = 150.0;
	int ratio = 3;
	int kernel_size = 3;
    int houghVote = 200;

	bool showHelp = true;
	bool fullScreen = false;
	const std::string windowName = "Camera Output";

	const std::string gst =  "nvcamerasrc ! video/x-raw(memory:NVMM), width=(int)1280, height=(int)720, \
							format=(string)I420, framerate=(fraction)120/1 ! \
							nvvidconv flip-method=0 ! video/x-raw, format=(string)BGRx ! \
							videoconvert ! video/x-raw, format=(string)BGR ! \
							appsink";

	cv::VideoCapture cap(gst);

	if(!cap.isOpened())
	{
		std::cout << "Failed to open camera." << std::endl;
		return -1;
	}

	cv::Mat edges;
	cv::namedWindow(windowName, 1);

	while(true)
	{
		cv::Mat src;
		cap >> src;  // Get a new Frame from the camera

		cv::cvtColor(src, edges, cv::COLOR_BGR2GRAY);
		cv::GaussianBlur(edges, edges, cv::Size(kernel_size, kernel_size), 1.5, 1.5);
		cv::Canny(edges, edges, lowThreshold, highThreshold, kernel_size);

		cv::Mat dst = src.clone();
        
		char key = (char)cv::waitKey(1);

		cv::Mat mask = cv::Mat::zeros(edges.rows, edges.cols, CV_32F);

		std::vector<cv::Point> vertices{cv::Point(src.cols, src.rows), \
										cv::Point(0, src.rows), \
										cv::Point(int(src.cols/2-10), int(2*src.rows/3)), \
										cv::Point(int(src.cols/2+10), int(2*src.rows/3))};

		cv::Mat masked_image;
		masked_image = region_of_interest(edges, vertices);

		std::vector<cv::Vec4i> lines;
		
		cv::HoughLinesP(masked_image, lines, 1.0f, (float) (CV_PI / 180.0f), 10, 5);
		cv::Mat line_image = cv::Mat::zeros(src.rows, src.cols, src.type());

		for (size_t i = 0; i < lines.size(); ++i)
		{
		    cv::Vec4i l = lines[i];
		    cv::line(line_image, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0, 0, 255), 3, cv::LINE_AA);
		}
		
        cv::line(line_image, cv::Point(int(src.cols/2), int(src.rows)), cv::Point(int(src.cols/2), int(src.rows-50)), cv::Scalar(0, 255, 0), 3, cv::LINE_AA);

		double alpha = 0.8; 
		double beta = 0.4;
		
		cv::addWeighted(src, alpha, line_image, beta, 0.0, dst);
		
		if (key == 27)
			break;

		window_management(dst, windowName, showHelp, fullScreen, key);

		cv::imshow(windowName, dst);
	}

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


