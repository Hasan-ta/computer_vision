#include "kitti_tools/custom_tools/libs/KittiOdometryIO.hpp"
#include <stdlib.h>
#include <opencv2/core/core.hpp>


int main(int argc, char** argv)
{
	try
	{
		perception::kitti::KittiOdometryIO odometryReader("/home/hasan/Desktop/computer_vision/data/kitti/odometry/aaa");
		while(1)
		{
			cv::Mat left, right;
			odometryReader.getNextFrame(left, right, true);
			
			cv::imshow("left", left);
			cv::waitKey(1);
			usleep(10);
		}
	}
	catch(std::exception& e)
	{
		std::cerr << e.what() << std::endl;
	}

	return 0;
}