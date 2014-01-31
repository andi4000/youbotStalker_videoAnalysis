#include <iostream>
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

bool verbose = true;

void getHSVThresholdedImg(const Mat& src, Mat& out, int h_lo, int h_hi, int s_lo, int s_hi, int v_lo, int v_hi){
	Mat matHSV;
	cvtColor(src, matHSV, CV_BGR2HSV);
	inRange(matHSV, Scalar(h_lo, s_lo, v_lo), Scalar(h_hi, s_hi, v_hi), out);
}

void doContourProcessing(const Mat& matSrc, const Mat& matThresh, Point& out_centroid, bool& out_gotIt, double& out_maxArea, Mat& out_mat)
{
	double minimalArea = 2000;
	Mat matOutput(matSrc);

	vector< vector<Point> > contours;
	findContours(matThresh, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

	vector<double> areas(contours.size());
	for (unsigned int i = 0; i < contours.size(); i++)
		areas[i] = contourArea(Mat(contours[i]));

	double max_area;
	Point maxPosition;
	minMaxLoc(Mat(areas), 0, &max_area, 0, &maxPosition);
	//ROS_INFO("biggest area = %f", max_area);

	bool got_it = false;
	if (max_area > minimalArea) got_it = true;

	// draw only the biggest contour
	drawContours(matOutput, contours, maxPosition.y, Scalar(0, 0, 255), CV_FILLED);

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
		circle(matOutput, image_centroid, 4, Scalar(255, 0, 0), -1, 8, 0);
		//ROS_INFO("image centroid (x, y) = (%d, %d)", image_centroid.x, image_centroid.y);

		// getting the relative centroid (middle point as 0,0)
		relative_centroid.x = image_centroid.x - (matSrc.cols / 2);
		relative_centroid.y = -image_centroid.y + (matSrc.rows / 2);
		if (verbose)
			cout<<"relative centroid (x,y) = ("<<relative_centroid.x<<", "<<relative_centroid.y<<") area "<<max_area<<endl;
			//printf("relative centroid (x, y) = (%d, %d) area %.2f", relative_centroid.x, relative_centroid.y, max_area);
	} else {
		if (verbose)
			cout<<"nothing"<<endl;
	}
	// GETTING THE CENTROID END

	//TODO: this is ugly
	out_centroid = relative_centroid;
	out_maxArea = max_area;
	out_gotIt = got_it;
	out_mat = matOutput;
}

int main (int argc, char** argv){
	VideoCapture cap;
	if (argc != 2){
		cout<<"Opening webcam"<<endl;
		cap.open(0);
	} else {
		cout<<"Opening "<<argv[1]<<endl;
		cap.open(argv[1]);
	}
	
	if (!cap.isOpened()){
		if (argc != 2)
			cout<<"Failed to open /dev/video0"<<endl;
		else
			cout<<"Failed to open file "<<argv[1]<<endl;
		return -1;
	}
	
	Mat tmp;
	cap.read(tmp);
	
	namedWindow("Original", CV_WINDOW_AUTOSIZE);
	namedWindow("Output", CV_WINDOW_AUTOSIZE);
	namedWindow("Threshold", CV_WINDOW_AUTOSIZE);
	
	moveWindow("Original", 0*tmp.cols, 0);
	moveWindow("Output", 1*tmp.cols, 0);
	moveWindow("Threshold", 2*tmp.cols, 0);
	
	//int h_lo = 23, h_hi = 60, s_lo = 68, s_hi = 174, v_lo = 81, v_hi = 228;
	int h_lo = 61, h_hi = 101, s_lo = 36, s_hi = 167, v_lo = 40, v_hi = 255;
	createTrackbar("H lo", "Threshold", &h_lo, 180);
	createTrackbar("H hi", "Threshold", &h_hi, 180);
	createTrackbar("S lo", "Threshold", &s_lo, 255);
	createTrackbar("S hi", "Threshold", &s_hi, 255);
	createTrackbar("V lo", "Threshold", &v_lo, 255);
	createTrackbar("V hi", "Threshold", &v_hi, 255);
	
	Mat frameOrig;
	while(1){
		if (!cap.read(frameOrig))
			break;
			
		Mat matThresh;
		getHSVThresholdedImg(frameOrig, matThresh, h_lo, h_hi, s_lo, s_hi, v_lo, v_hi);
		
		erode(matThresh, matThresh, Mat());
		dilate(matThresh, matThresh, Mat());
		erode(matThresh, matThresh, Mat());
		/**
		Mat matOutput(frameOrig);
		Point relative_centroid(0,0);
		double max_area;
		bool got_it;
		doContourProcessing(frameOrig, matThresh, relative_centroid, got_it, max_area, matOutput);
		*/
		imshow("Original", frameOrig);
		imshow("Output", matThresh);
		
		char key = waitKey(27);
		if (key == 27)
			break;
	}

	
//	namedWindow("Display", CV_WINDOW_AUTOSIZE);
	//imshow("Display", image);

//	waitKey(0);

	return 0;


}
