#include "kitti_tools/custom_tools/flow_reader/flow_stereo_io.h"
#include "kitti_tools/custom_tools/flow_reader/kitti_flow_calibration_reader.h"
#include "MonOdometry.h"
#include <opencv2/core/core.hpp>


int main(int argc, char** argv)
{
	perception::kitti::KittiFlowStereoIO2015 flow_reader("/home/hasan/Desktop/computer_vision/data/kitti/flow_stereo/data_scene_flow",
			"/home/hasan/Desktop/computer_vision/data/kitti/flow_stereo/calibration");
	perception::odometry::MonOdometry odometer;

	cv::Mat f1, f2;

	while(flow_reader.get_next_mono_frames(f1,f2))
	{
		perception::kitti::FlowCamCalibParams left_cam_params;
		flow_reader.get_current_frame_cam2cam_calibration(left_cam_params);

		odometer.setFrames(f1,f2);
		cv::Mat K;
		left_cam_params.getK(K);
		odometer.setCameraMatrix(K);
		odometer.calculateMotion();
	}

	return 0;
}