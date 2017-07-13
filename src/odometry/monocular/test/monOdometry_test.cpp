#include "kitti_tools/custom_tools/libs/KittiOdometryIO.hpp"
#include "libs/MonOdometry.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <stdlib.h>
#include <exception>

cv::Mat image2 = cv::Mat::zeros(1000,1000,CV_8U);

void visualize_motion(const cv::Mat& newPos)
{
	cv::Mat trans= (cv::Mat_<double>(2,1) << newPos.at<double>(0,3), newPos.at<double>(2,3));
	cv::Mat newPosition(2,1,CV_64F);
	cv::Mat latestPosition = (cv::Mat_<double>(2,1) << 500, 500);
	newPosition = latestPosition - trans/2;

	cv::Mat newLineImage = cv::Mat::zeros(1000,1000,CV_8U);
	cv::Point p1(0, 0);
	cv::Point p2(newPosition.at<double>(0,0), newPosition.at<double>(1,0));
	std::cerr << "latestPosition: " << p1 << std::endl;
	std::cerr << "newPosition: " << p2 << std::endl;
	std::cerr << "trans: " << trans << std::endl;
	circle(newLineImage, p2, 2, cv::Scalar(255,0,0), 1);
	addWeighted(image2, 1.0, newLineImage, 1.0, 0.0, image2);
	cv::imshow("result2", image2);
	cv::waitKey(1);



}

int main(int argc, char** argv)
{
	try
	{
		perception::kitti::KittiOdometryIO odometryReader("/home/hasan/Desktop/computer_vision/data/kitti/odometry/dataset/");
		perception::odometry::MonOdometry odometer;
		cv::Mat left1, right1;
		cv::Mat left2, right2;
		while(1)
		{
			odometryReader.getNextFrame(left1, right1);
			odometryReader.getNextFrame(left2, right2);
			odometer.setFrames(left1,left2);
			cv::Mat P0, P1;
			odometryReader.getCalibration(P0,P1);
			cv::Mat K0 = P0(cv::Rect(0,0,3,3));
			// cv::Mat K0 = (cv::Mat_<double>(3,3) << 9.842439e+02, 0.000000e+00, 6.900000e+02, 0.000000e+00, 9.808141e+02, 2.331966e+02, 0.000000e+00, 0.000000e+00, 1.000000e+00);
			odometer.setCameraMatrix(K0);
			odometer.calculateMotion();

			cv::Mat gTPose(3,4,CV_64F);
			odometryReader.getGroundTruthPose(gTPose);
			// cv::imshow("image", left1);
			// visualize_motion(gTPose);
			usleep(40*1000);
		}

		return 0;

	}
	catch(std::exception& e)
	{
		std::cerr << e.what() << std::endl;
		return -1;
	}
	
}