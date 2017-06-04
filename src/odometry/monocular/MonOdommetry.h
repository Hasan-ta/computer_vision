#ifndef MonOdometry_H
#define MonOdometry_H


#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/contrib/contrib.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/features2d/features2d.hpp>



namespace perception{
	class MonOdometry{
	public:
		MonOdometry();

		void setFrames(cv::Mat f1, cv::Mat f2);

		void calculateMotion();


	private:
		cv::Mat _frame1, _frame2;
		bool _new_frames = false;


	}
}

#endif