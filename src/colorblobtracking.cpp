#include "ros/ros.h"
#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"

#include "std_msgs/Bool.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float32.h"

/**
 * //TODO
 * - make everything standardized CPP!
 * - put everything into modular functions!
 * 
 * things that are still in C
 * - getHSVThresholdedImg function
 * - camera capture
 */

IplImage* getHSVThresholdedImg(IplImage* img, int lo_h, int lo_s, int lo_v, int hi_h, int hi_s, int hi_v);

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
	
	// ROS MESSAGE BEGIN
	// Ref: http://www.ros.org/wiki/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29
	ros::init(argc, argv, "object_tracking_node");
	ros::NodeHandle n;
	ros::Publisher object_detected_pub = n.advertise<std_msgs::Bool>("object_tracking/object_detected", 1000);
	ros::Publisher x_pos_pub = n.advertise<std_msgs::Int32>("object_tracking/x_pos", 1000);
	ros::Publisher y_pos_pub = n.advertise<std_msgs::Int32>("object_tracking/y_pos", 1000);
	ros::Publisher area_pub = n.advertise<std_msgs::Float32>("object_tracking/area", 1000);
	ros::Rate loop_rate(50);
	// ROS MESSAGE END
	
	int capSizeX = 640;
	int capSizeY = 480;
	
	namedWindow("Webcam", CV_WINDOW_AUTOSIZE);
	namedWindow("Output", CV_WINDOW_AUTOSIZE);
	moveWindow("Webcam", 0*capSizeX, 0);
	moveWindow("Output", 1*capSizeX, 0);
	
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
	
	
	// ROS SIGINT handler
	while(ros::ok()){
		IplImage* srcImg = cvQueryFrame(capture);
		if (!srcImg) {
			ROS_ERROR("Failed to get frame from camera!");
			return 1;			
		}

		IplImage* threshImg = getHSVThresholdedImg(srcImg, lo_h, lo_s, lo_v, hi_h, hi_s, hi_v);
		
		// CONTOUR TEST PART2 BEGIN
		// source: http://www.bytefish.de/blog/extracting_contours_with_opencv
		Mat threshMat(threshImg);
		Mat srcMat(srcImg);
		
		vector< vector<Point> > contours;
		findContours(threshMat, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
		
		vector<double> areas(contours.size());
		for (unsigned int i = 0; i < contours.size(); i++)
			areas[i] = contourArea(Mat(contours[i]));
		
		double max_area;
		Point maxPosition;
		minMaxLoc(Mat(areas), 0, &max_area, 0, &maxPosition);
		//ROS_INFO("biggest area = %f", max_area);
		
		bool got_it = false;
		if (max_area > 2000) got_it = true;
		
		if (got_it)
			ROS_INFO("got it");
		else
			ROS_INFO("nothing");
		// draw only the biggest contour
		drawContours(srcMat, contours, maxPosition.y, Scalar(0, 0, 255), CV_FILLED);
		
		// GETTING THE CENTROID BEGIN
		Point image_centroid(0, 0);
		Point relative_centroid(0, 0);
		if (got_it){
			vector<Moments> mu(contours.size());
			for ( unsigned int i = 0; i < contours.size(); i++ ) {
				mu[i] = moments(contours[i], false);
			}
			
			vector<Point> mc(contours.size());
			for ( unsigned int i = 0; i < contours.size(); i++ ) {
				mc[i] = Point( mu[i].m10/mu[i].m00, mu[i].m01/mu[i].m00 );
			}
			
			// draw the centroid of the biggest contour
			image_centroid = mc[maxPosition.y];
			circle(srcMat, image_centroid, 4, Scalar(255, 0, 0), -1, 8, 0);
			//ROS_INFO("image centroid (x, y) = (%d, %d)", image_centroid.x, image_centroid.y);
			
			// getting the relative centroid (middle point as 0,0)
			relative_centroid.x = image_centroid.x - (capSizeX / 2);
			relative_centroid.y = -image_centroid.y + (capSizeY / 2);
			ROS_INFO("relative centroid (x, y) = (%d, %d) area %.2f", relative_centroid.x, relative_centroid.y, max_area);
		}
		// GETTING THE CENTROID END
		
		imshow("Output", threshMat);
		imshow("Webcam", srcMat);
		// CONTOUR TEST PART2 END
				
		// ROS MESSAGE BEGIN
		std_msgs::Bool msg_obj_det;
		std_msgs::Int32 msg_x_pos, msg_y_pos;
		std_msgs::Float32 msg_area;
		
		msg_obj_det.data = got_it;
		msg_x_pos.data = relative_centroid.x;
		msg_y_pos.data = relative_centroid.y;
		msg_area.data = max_area;
		
		object_detected_pub.publish(msg_obj_det);
		x_pos_pub.publish(msg_x_pos);
		y_pos_pub.publish(msg_y_pos);
		area_pub.publish(msg_area);
		
		ros::spinOnce();
		loop_rate.sleep();
		// ROS MESSAGE END
		
		// press ESC to exit
		if ( (waitKey(10) & 255) == 27 ) break;
	}
	
	cvReleaseCapture(&capture);
	destroyAllWindows();
	
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
