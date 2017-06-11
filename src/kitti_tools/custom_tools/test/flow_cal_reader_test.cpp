#include "kitti_flow_calibration_reader.h"
#include <exception>

int main(int argc, char** argv)
{
	perception::kitti::FlowCalibrationReader reader("/home/hasan/Desktop/computer_vision/data/kitti/flow_stereo/calibration");

	try
	{
		perception::kitti::cam2camCalibrationParams params = reader.get_cam2cam_calibration(0);
		std::cout << "S_00: " << std::endl << params.four_cameras_params[0].getSize() << std::endl;
		std::cout << "K_00: " << std::endl << params.four_cameras_params[0].getK() << std::endl;
		std::cout << "D_00: " << std::endl << params.four_cameras_params[0].getD() << std::endl;
		std::cout << "P_00: " << std::endl << params.four_cameras_params[0].getP() << std::endl;
	}
	catch(std::exception& e)
	{
		std::cout << e.what() << std::endl;
		return -1;
	}

	return 0;


}