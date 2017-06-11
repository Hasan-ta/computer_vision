#include "kitti_flow_calibration_reader.h"
#include <exception>

int main(int argc, char** argv)
{
	perception::kitti::FlowCalibrationReader reader("/home/hasan/Desktop/computer_vision/data/kitti/flow_stereo/calibration");

	try
	{
		perception::kitti::cam2camFlowCalibrationParams params;
		reader.get_cam2cam_calibration(0,params);
		cv::Mat K , D, ExtMat, R_rect, P_rect;

		for (int i = 0; i < 4; ++i)
		{
			params.four_cameras_params[i].getK(K);
			params.four_cameras_params[i].getD(D);
			params.four_cameras_params[i].getExtMat(ExtMat);
			params.four_cameras_params[i].getR_rect(R_rect);
			params.four_cameras_params[i].getP_rect(P_rect);
			std::cout << std::endl << "S_0" << i << ": " << std::endl << params.four_cameras_params[i].getSize() << std::endl;
			std::cout << std::endl << "K_0" << i << ": " << std::endl << K << std::endl;
			std::cout << std::endl << "D_0" << i << ": " << std::endl << D << std::endl;
			std::cout << std::endl << "ExtMat_0" << i << ": " << std::endl << ExtMat << std::endl;
			std::cout << std::endl << "S_rect_0" << i << ": " << std::endl << params.four_cameras_params[i].getSize_rect() << std::endl;
			std::cout << std::endl << "R_rect_0" << i << ": " << std::endl << R_rect << std::endl;
			std::cout << std::endl << "P_rect_0" << i << ": " << std::endl << P_rect << std::endl;
			std::cout << std::string(30,'-') << std::endl;

		}
	}
	catch(std::exception& e)
	{
		std::cout << e.what() << std::endl;
		return -1;
	}

	return 0;


}