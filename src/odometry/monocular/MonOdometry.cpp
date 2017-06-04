#include "MonOdometry.h"

namespace perception{

	void MonOdometry::setFrames(cv::Mat f1, cv::Mat f2)
	{
		_frame1 = f1;
		_frame2 = f2;
		_new_frames = true;
	}

	void calculateMotion()
	{
		if(!_new_frames)
			std::cerr << "Motion has already been evaluated! use setFrames(f1,f2) to input new frames!" << std::endl;
		else
		{
			/* Workflow:
			 * 	1-Detect Features in frame1 using shi-tomasi
			 *	2- Use Pyramidal KLT to estimate motion
			 * 	3- Estimate Fundamental Matrix F (RNASC, 8-Point, 5-Point )
			 *	4- Compute Essential Matrix from F and Calibration Matrix K
			 *	5-  
		}

	}
}
