#include "ros/ros.h"

//#include "opencv2/opencv.hpp"
//#include "opencv2/highgui/highgui.hpp"
#include "opencv/cv.h"
#include "opencv/highgui.h"

int main(int argc, char** argv)
{
	CvCapture* capture = cvCaptureFromCAM(0);
	
	if (!capture) {
		ROS_ERROR("Failed to open camera!");
		return 1;
	} else {
		ROS_INFO("Opening webcam");
	}
	
	cvNamedWindow("webcam", CV_WINDOW_AUTOSIZE);
	
	while(1){
		IplImage* srcImg = cvQueryFrame(capture);
		cvShowImage("webcam", srcImg);
		
		// press ESC to exit
		if ( (cvWaitKey(10) & 255) == 27 ) break;
	}
	
	cvReleaseCapture(&capture);
	cvDestroyWindow("webcam");
	
	return 0;
}

