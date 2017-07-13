#include "kitti_tools/custom_tools/libs/KittiOdometryIO.hpp"
#include <stdlib.h>
#include <opencv2/core/core.hpp>


int main(int argc, char** argv)
{
	try
	{
		perception::kitti::KittiOdometryIO odometryReader("/home/hasan/Desktop/computer_vision/data/kitti/odometry/dataset/");
		odometryReader.seek(0,4000);
		while(1)
		{
			cv::Mat left, right;
			odometryReader.getNextFrame(left, right, true);
			
			cv::imshow("left", left);
			cv::imshow("right", right);
			cv::waitKey(1);
			usleep(1000);

			if(odometryReader.endOfSequence())
			{
				cv::Mat P0, P1;
				odometryReader.getCalibration(P0,P1);
				std::cout << std::endl << "P0: " << std::endl << P0 << std::endl;
				std::cout << std::endl << "P1: " << std::endl << P1 << std::endl;

				break;
			}
		}
		odometryReader.seek(0,10);

		cv::Mat gT(12,1,CV_64F);
		odometryReader.getGroundTruthPose(gT);

		std::cout << "groundTruth: " << gT << std::endl;
	}
	catch(std::exception& e)
	{
		std::cerr << e.what() << std::endl;
	}

	return 0;
}