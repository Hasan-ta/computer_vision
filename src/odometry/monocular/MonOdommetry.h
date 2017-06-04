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
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/contrib/contrib.hpp>




namespace perception{
namespace odometry{

	class MonOdometry{
	public:
		MonOdometry();

		void setFrames(cv::Mat, cv::Mat);

		void setMinCornerDistance(const int&);

		void calculateMotion();


	private:
		cv::Mat frame1_, frame2_;
		bool new_frames_ = false;		

		// Feature Detector Params
		cv::vector<cv::Point2f> corners_;
		int max_corners_, min_corner_distance_, detector_block_size_;
		cv::Mat detector_roi_;
		bool use_harris_;
		float corners_quality_;

		// 


	};
}
}

#endif