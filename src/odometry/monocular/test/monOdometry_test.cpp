#include "kitti_tools/custom_tools/libs/KittiOdometryIO.hpp"
#include "libs/MonOdometry.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <stdlib.h>
#include <exception>

cv::Mat image2 = cv::Mat::zeros(1000,1000,CV_8UC3);

void visualize_motion(const cv::Mat& newPos, const cv::Mat& p)
{
	cv::Mat latestPosition = (cv::Mat_<double>(2,1) << 500, 500);

	cv::Mat trans= (cv::Mat_<double>(2,1) << newPos.at<double>(0,3), newPos.at<double>(2,3));
	cv::Mat newPosition(2,1,CV_64F);
	newPosition = latestPosition - trans/2;

	cv::Mat trans_c= (cv::Mat_<double>(2,1) << p.at<double>(0,3), p.at<double>(2,3));
	cv::Mat newPosition_c(2,1,CV_64F);
	newPosition_c = latestPosition - trans_c/2;

	cv::Mat newLineImage = cv::Mat::zeros(1000,1000,CV_8UC3);
	circle(newLineImage, cv::Point(newPosition.at<double>(0), newPosition.at<double>(1)), 2, cv::Scalar(255,0,0), 1);
	circle(newLineImage, cv::Point(newPosition_c.at<double>(0), newPosition_c.at<double>(1)), 2, cv::Scalar(0,0,255), 1);
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
		cv::Mat gTPose(3,4,CV_64F);
		cv::Mat gTPose_prev = cv::Mat::zeros(3,4,CV_64F);

		cv::Mat R_f = cv::Mat::zeros(3,3,CV_64F);
		cv::Mat t_f = cv::Mat::zeros(3,1,CV_64F);

		int counter = 0;
		while(1)
		{
			if(counter == 0)
			{
				odometryReader.getNextFrame(left1, right1);
				odometryReader.getNextFrame(left2, right2);
			}
			else
			{
				left1 = left2.clone();
				odometryReader.getNextFrame(left2, right2);
			}
			odometer.setFrames(left1,left2);
			cv::Mat P0, P1;
			odometryReader.getCalibration(P0,P1);
			cv::Mat K0 = P0(cv::Rect(0,0,3,3));
			// std::cout << "K0: " << std::endl << K0 << std::endl;
			// cv::Mat K0 = (cv::Mat_<double>(3,3) << 9.842439e+02, 0.000000e+00, 6.900000e+02, 0.000000e+00, 9.808141e+02, 2.331966e+02, 0.000000e+00, 0.000000e+00, 1.000000e+00);
			odometer.setCameraMatrix(K0);
			cv::Mat R, t, p;

			odometer.calculateMotion(R,t);

			std::cout << "counter: " << counter << std::endl;
			cv::Mat gTPose(3,4,CV_64F);
			odometryReader.getGroundTruthPose(gTPose);
			double x, x_prev, y, y_prev, z, z_prev;
			x = gTPose.at<double>(0,3); x_prev = gTPose_prev.at<double>(0,3);
			y = gTPose.at<double>(1,3); y_prev = gTPose_prev.at<double>(1,3);
			z = gTPose.at<double>(2,3); z_prev = gTPose_prev.at<double>(2,3);
			std::cout << "gTpose: " << std::endl << gTPose << std::endl;
			std::cout << "gTpose_prev: " << std::endl << gTPose_prev << std::endl;
			gTPose_prev = gTPose.clone();

			double scale = sqrtf(powf(x-x_prev,2) + powf(y-y_prev,2) + 
							powf(z-z_prev,2));

			// R = cv::Mat::eye(3,3,CV_64F);
			if(counter == 0)
			{
				t_f = t*scale;
				R_f = R;
			}
			else
			{
				t_f = t_f + scale*(R_f*t);
				R_f = R*R_f;	
			}

			std::cout << "scale: " << scale << std::endl;
			std::cout << "t: " << std::endl << t << std::endl;
			std::cout << "R: " << std::endl << R << std::endl;
			std::cout << std::endl << "R_true: " << std::endl << gTPose(cv::Rect(0,0,3,3)) << std::endl;
			std::cout << std::endl << "t_true: " << std::endl << gTPose.col(3) << std::endl;
			std::cout << "t_f: " << std::endl << t_f << std::endl;
			std::cout << "R_f: " << std::endl << R_f << std::endl;
			std::cout << "----------------------------------------------------------------" << std::endl;

			counter++;
			cv::hconcat(R_f,t_f,p);

			// cv::imshow("image", left1);
			visualize_motion(gTPose, p);
			usleep(10*1000);
		}

		return 0;

	}
	catch(std::exception& e)
	{
		std::cerr << e.what() << std::endl;
		return -1;
	}
	
}