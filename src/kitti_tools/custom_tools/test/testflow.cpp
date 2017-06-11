#include "flow_stereo_io.h"
#include <exception>

using namespace cv;
using namespace perception::kitti;
int main(int argc, char** argv)
{
	try{

	KittiFlowStereoIO2015 stereo_reader("/home/hasan/Desktop/computer_vision/data/kitti/flow_stereo/data_scene_flow",
		"/home/hasan/Desktop/computer_vision/data/kitti/flow_stereo/calibration");	
	FlowCamCalibParams left_cal, right_cal;
	stereo_reader.get_current_frame_cam2cam_calibration(left_cal, true);
	stereo_reader.get_current_frame_cam2cam_calibration(right_cal, false);
	cv::Mat frame1, frame2;
	stereo_reader.get_specific_mono_frames(frame1,frame2,100);

	namedWindow( "Display window", WINDOW_AUTOSIZE );// Create a window for display.
    imshow( "frame1", frame1 );                   // Show our image inside it.

    waitKey(0);

    imshow( "frame2",frame2 );     

    waitKey(0);

    cv::Mat K, D, ExtMat, R_rect, P_rect;

	left_cal.getK(K);
	left_cal.getD(D);
	left_cal.getExtMat(ExtMat);
	left_cal.getR_rect(R_rect);
	left_cal.getP_rect(P_rect);
    std::cout << std::endl << "S_l" <<  ": " << std::endl << left_cal.getSize() << std::endl;
	std::cout << std::endl << "K_l" <<  ": " << std::endl << K << std::endl;
	std::cout << std::endl << "D_l" <<  ": " << std::endl << D << std::endl;
	std::cout << std::endl << "ExtMat_l" <<  ": " << std::endl << ExtMat << std::endl;
	std::cout << std::endl << "S_rect_l" <<  ": " << std::endl << left_cal.getSize_rect() << std::endl;
	std::cout << std::endl << "R_rect_l" <<  ": " << std::endl << R_rect << std::endl;
	std::cout << std::endl << "P_rect_l" <<  ": " << std::endl << P_rect << std::endl;
	std::cout << std::string(30,'-') << std::endl;


	right_cal.getK(K);
	right_cal.getD(D);
	right_cal.getExtMat(ExtMat);
	right_cal.getR_rect(R_rect);
	right_cal.getP_rect(P_rect);
    std::cout << std::endl << "S_r" << ": " << std::endl << right_cal.getSize() << std::endl;
	std::cout << std::endl << "K_r" <<  ": " << std::endl << K << std::endl;
	std::cout << std::endl << "D_r" <<  ": " << std::endl << D << std::endl;
	std::cout << std::endl << "ExtMat_r" <<  ": " << std::endl << ExtMat << std::endl;
	std::cout << std::endl << "S_rect_r" <<  ": " << std::endl << right_cal.getSize_rect() << std::endl;
	std::cout << std::endl << "R_rect_r" <<  ": " << std::endl << R_rect << std::endl;
	std::cout << std::endl << "P_rect_r" <<  ": " << std::endl << P_rect << std::endl;
	std::cout << std::string(30,'-') << std::endl;

    return 0;

	}
	catch (std::exception& e)
	{
		std::cout << e.what() << std::endl;
	}
}