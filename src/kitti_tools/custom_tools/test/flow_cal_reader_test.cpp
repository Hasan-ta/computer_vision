#include "kitti_flow_calibration_reader.h"
#include <exception>

int main(int argc, char** argv)
{
	perception::kitti::FlowCalibrationReader reader("/home/hasan/Desktop/computer_vision/data/kitti/flow_stereo/calibration");

	try
	{
		reader.get_cam2cam_calibration(0);
	}
	catch(std::exception& e)
	{
		std::cout << e.what() << std::endl;
		return -1;
	}

	return 0;


}