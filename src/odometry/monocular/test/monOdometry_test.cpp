#include "custom_tools/flow_stereo_io.h"
#include "MonOdometry.h"
#include <opencv2/core/core.hpp>


int main(int argc, char** argv)
{
	perception::kitti::KittiFlowStereoIO2015 flow_reader("/home/hasan/Desktop/computer_vision/data/kitti/flow_stereo/data_scene_flow");
	perception::odometry::MonOdometry odometer;

	cv::Mat f1, f2;

	while(flow_reader.get_next_mono_frames(f1,f2))
	{
		odometer.setFrames(f1,f2);
		odometer.calculateMotion();
	}

	return 0;
}