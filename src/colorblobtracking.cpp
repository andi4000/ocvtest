#include "ros/ros.h"
#include "opencv/cv.h"
#include "opencv/highgui.h"

//TODO:
// - make centroid calculation more robust
// - implement publishing messages (x and y position of the blob centroid), ref: ROS beginner tutorials

IplImage* getHSVThresholdedImg(IplImage* img, int lo_h, int lo_s, int lo_v, int hi_h, int hi_s, int hi_v);
CvPoint calculateCentroid(IplImage* threshedImg);

using namespace cv;

int main(int argc, char** argv)
{
	CvCapture* capture = cvCaptureFromCAM(0);
	
	if (!capture) {
		ROS_ERROR("Failed to open camera!");
		return 1;
	} else {
		ROS_INFO("Opening webcam");
	}
	
	
	cvNamedWindow("Webcam", CV_WINDOW_AUTOSIZE);
	cvNamedWindow("Output", CV_WINDOW_AUTOSIZE);
	
	// HSV max and min values in opencv
	//int lo_h = 0, lo_s = 0, lo_v = 0, hi_h = 180, hi_s = 255, hi_v = 255;
	// this is for green
	int lo_h = 23, lo_s = 68, lo_v = 81, hi_h = 60, hi_s = 174, hi_v = 228;
	createTrackbar("Lo H", "Output", &lo_h, 255);
	createTrackbar("Lo S", "Output", &lo_s, 255);
	createTrackbar("Lo V", "Output", &lo_v, 255);
	createTrackbar("Hi H", "Output", &hi_h, 180);
	createTrackbar("Hi S", "Output", &hi_s, 255);
	createTrackbar("Hi V", "Output", &hi_v, 255);
	
	while(1){
		IplImage* srcImg = cvQueryFrame(capture);
		if (!srcImg) {
			ROS_ERROR("Failed to get frame from camera!");
			return 1;			
		}

		IplImage* threshImg = getHSVThresholdedImg(srcImg, lo_h, lo_s, lo_v, hi_h, hi_s, hi_v);
		
		// Centroid calculation
		CvPoint centroid = calculateCentroid(threshImg);
		
		cvCircle(srcImg, centroid, 5, cvScalar(0, 0, 255), 5);
		
		cvShowImage("Webcam", srcImg);
		cvShowImage("Output", threshImg);
		
		// press ESC to exit
		if ( (cvWaitKey(10) & 255) == 27 ) break;
	}
	
	cvReleaseCapture(&capture);
	cvDestroyWindow("Webcam");
	cvDestroyWindow("Output");
	
	return 0;
}

// Ref: http://www.aishack.in/2010/07/tracking-colored-objects-in-opencv/
IplImage* getHSVThresholdedImg(IplImage* img, int lo_h, int lo_s, int lo_v, int hi_h, int hi_s, int hi_v)
{
	IplImage* imgHSV = cvCreateImage(cvGetSize(img), 8, 3);
	cvCvtColor(img, imgHSV, CV_BGR2HSV);
	
	IplImage* imgThresh = cvCreateImage(cvGetSize(img), 8, 1);
	cvInRangeS(imgHSV, cvScalar(lo_h, lo_s, lo_v), cvScalar(hi_h, hi_s, hi_v), imgThresh);
	
	cvErode(imgThresh, imgThresh);
	
	cvReleaseImage(&imgHSV);
	return imgThresh;
}

// Ref: http://stackoverflow.com/questions/8143414/centroid-of-contour-object-in-opencv-in-c
CvPoint calculateCentroid(IplImage* threshedImg)
{
	int centroid_x = 0, centroid_y = 0;
	CvMoments m;
	double m00, m01, m10;
	
	cvMoments(threshedImg, &m, true);
	m00 = cvGetSpatialMoment(&m, 0, 0);
	m01 = cvGetSpatialMoment(&m, 0, 1);
	m10 = cvGetSpatialMoment(&m, 1, 0);
	
	centroid_x = (int) m10 / m00;
	centroid_y = (int) m01 / m00;
	
	return cvPoint(centroid_x, centroid_y);
}
