#include "ros/ros.h"
#include "opencv/cv.h"
#include "opencv/highgui.h"

#include <iostream>
#include <stdio.h>
#include <stdlib.h>

//TODO:
// - make centroid calculation more robust (hint: simple blob detector, findcontour)
// - implement publishing messages (x and y position of the blob centroid), ref: ROS beginner tutorials

IplImage* getHSVThresholdedImg(IplImage* img, int lo_h, int lo_s, int lo_v, int hi_h, int hi_s, int hi_v);
CvPoint calculateCentroid(IplImage* threshedImg);

using namespace cv;
using namespace std;

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
	createTrackbar("Lo H", "Output", &lo_h, 180);
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
		
		//TODO: THIS PART IS NOT RELIABLE
		//TODO: http://www.bytefish.de/blog/extracting_contours_with_opencv
		/**
		// contour test begin
		// Page 243-244
		// http://stackoverflow.com/questions/6044119/opencv-cvfindcontours-how-do-i-separate-components-of-a-contour
		// http://books.google.de/books?id=seAgiOfu2EIC&pg=PA234&lpg=PA234&dq=cvFindContours&source=bl&ots=hSJ3eleCKa&sig=tdP8PSgr1azn2Im8rmqSGhzonBU&hl=en&sa=X&ei=QW1MUc3TCIXltQaB94CwAQ&ved=0CGUQ6AEwBQ#v=onepage&q=cvFindContours&f=false
		CvMemStorage* storage = cvCreateMemStorage();
		CvSeq* first_contour = NULL;
		CvSeq* first_polygon = NULL;
		
		//int Nc = cvFindContours(threshImg, storage, &first_contour, sizeof(CvContour), CV_RETR_LIST);
		int Nc = cvFindContours(threshImg, storage, &first_contour, sizeof(CvContour), CV_RETR_EXTERNAL);
		
		first_polygon = cvApproxPoly(first_contour, sizeof(CvContour), storage, CV_POLY_APPROX_DP, 2, 1);
		
		ROS_INFO("Total contours detected: %d", Nc);
		
		
		CvSeq* biggest_part;
		double area_buf = 0;
		double biggest_area = 0;
		// find the biggest contour area
		for( CvSeq* c = first_contour; c != NULL; c = c->h_next) {
			biggest_area = cvContourArea(c);
			if (biggest_area > area_buf){
				area_buf = biggest_area;
				biggest_part = c;
			}
		}
		ROS_INFO("Biggest area: %d", biggest_area);
		
		
		for( CvSeq* c = biggest_part; c != NULL; c = c->h_next) {
		//for( CvSeq* c = first_polygon; c != NULL; c = c->h_next) {
			//cvDrawContours(srcImg, c, CVX_RED, CVX_BLUE, 0, 2, 8);
			cvDrawContours(srcImg, c, CV_RGB(255, 0, 0), CV_RGB(0, 0, 0), -1, CV_FILLED, 8);
		}
		*/
		// contour test end
		
		
		// CONTOUR TEST PART2 BEGIN
		// source: http://www.bytefish.de/blog/extracting_contours_with_opencv
		Mat threshMat(threshImg);
		Mat srcMat(srcImg);
		
		vector< vector<Point> > contours;
		findContours(threshMat, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
		
		vector<double> areas(contours.size());
		for (int i = 0; i < contours.size(); i++)
			areas[i] = contourArea(Mat(contours[i]));
		
		double max_area;
		Point maxPosition;
		minMaxLoc(Mat(areas), 0, &max_area, 0, &maxPosition);
		ROS_INFO("biggest area = %f", max_area);
		
		// draw only the biggest contour
		drawContours(srcMat, contours, maxPosition.y, Scalar(0, 0, 255), CV_FILLED);
		
		//TODO:
		// - find the centroid
		// - output the centroid x y and area
		
		imshow("Webcam", srcMat);
		imshow("Output", threshMat);
		// CONTOUR TEST PART2 END
		
		
		
		// Centroid calculation and display
		//CvPoint centroid = calculateCentroid(threshImg);
		//cvCircle(srcImg, centroid, 5, cvScalar(0, 0, 255), 5);
		
		//cvShowImage("Webcam", srcImg);
		//cvShowImage("Output", threshImg);
		
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
