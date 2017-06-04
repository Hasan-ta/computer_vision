#include "flow_stereo_io.h"
#include <exception>
#include "custom_exceptions.h"


using namespace cv;
int main(int argc, char** argv)
{
	try{

	perception::KittiFlowStereoIO2015 stereo_reader("/home/hasan/Desktop/computer_vision/data/kitti/flow_stereo/data_scene_flow");	
	
	cv::Mat frame1, frame2;
	stereo_reader.get_specific_mono_frames(frame1,frame2,100);

	namedWindow( "Display window", WINDOW_AUTOSIZE );// Create a window for display.
    imshow( "frame1", frame1 );                   // Show our image inside it.

    waitKey(0);

    imshow( "frame2",frame2 );     

    waitKey(0);

    return 0;

	}
	catch (exception& e)
	{
		std::cout << e.what() << std::endl;
	}
}