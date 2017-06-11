#include "MonOdometry.h"

#define DEBUG

#ifdef DEBUG

void draw_features_on_image(const cv::Mat& gray, const cv::Vector<cv::Point2f>& corners_)
{
	cv::Mat circles_image(gray.rows,gray.cols, CV_8UC3, cv::Scalar(0,0,0));
	for (uint32_t i = 0; i < corners_.size(); ++i)
		cv::circle(circles_image, corners_[i], 3, CV_RGB(0, 255, 0), 2, 2, 0);
	cv::Mat vis;
	std::cerr << gray.size() << ", " << circles_image.size() << std::endl;
	cv::Mat gray_vis;
	cv::cvtColor(gray, gray_vis, CV_GRAY2BGR);
	addWeighted(gray_vis, 1.0, circles_image, 1.0, 0.0, vis);
	cv::imshow("detected corners", vis);
	cv::waitKey(0);
}

void draw_flow_arrows(const cv::Mat& gray, const cv::Vector<cv::Point2f>& pts1, const cv::Vector<cv::Point2f>& pts2)
{
	cv::Mat arrows_image(gray.rows,gray.cols, CV_8UC3, cv::Scalar(0,0,0));
	for (uint32_t i = 0; i < pts1.size(); ++i)
		arrowedLine(arrows_image, pts1[i], pts2[i], CV_RGB(255, 0, 0), 3, 8, 0, 0.1);
	cv::Mat vis;
	cv::Mat gray_vis;
	cv::cvtColor(gray, gray_vis, CV_GRAY2BGR);
	addWeighted(gray_vis, 1.0, arrows_image, 1.0, 0.0, vis);
	cv::imshow("detected corners", vis);
	cv::waitKey(0);
}

#endif

namespace perception{
namespace odometry{

	MonOdometry::MonOdometry():
		max_corners_(2000),
		corners_quality_(0.01),
		min_corner_distance_(10),
		detector_block_size_(3),
		use_harris_(false)
	{}

	void MonOdometry::setFrames(cv::Mat f1, cv::Mat f2)
	{
		frame1_ = f1;
		frame2_ = f2;
		new_frames_ = true;
	}

	void MonOdometry::setMinCornerDistance(const int& dist)
	{
		min_corner_distance_ = dist;
	}

	void MonOdometry::extractFeatures()
	{
		detector_roi_ = cv::Mat(cv::Size(0,0), CV_8UC1);
		corners_.clear();
		setMinCornerDistance(20);
		cv::goodFeaturesToTrack(gray_f1, corners_, max_corners_, corners_quality_,
			min_corner_distance_, detector_roi_,
			detector_block_size_, use_harris_);
	}

	void MonOdometry::trackFeatures()
	{
		tracked_features_.clear();
	  	std::vector<float> err;
	  	std::vector<uchar> status;					
		cv::Size winSize=cv::Size(21,21);																								
		cv::TermCriteria termcrit = cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 0.01);

		cv::calcOpticalFlowPyrLK(gray_f1, gray_f2, corners_, tracked_features_, status, err, winSize, 3, termcrit, 0, 0.001);

		int indexCorrection = 0;
	  	for( int i=0; i<status.size(); i++)
 		{  
 			cv::Point2f pt = tracked_features_.at(i- indexCorrection);
 			if ((status.at(i) == 0)||(pt.x<0)||(pt.y<0))	
 			{
     		  	if((pt.x<0)||(pt.y<0))	
     		  	{
     		  		status.at(i) = 0;
     		  	}
	     		corners_.erase (corners_.begin() + i - indexCorrection);
	     		tracked_features_.erase (tracked_features_.begin() + i - indexCorrection);
	     		indexCorrection++;
 			}

 		}
	}

	void MonOdometry::calculateMotion()
	{
		if(!new_frames_)
			std::cerr << "Motion has already been evaluated! use setFrames(f1,f2) to input new frames!" << std::endl;
		else
		{
			/* Workflow:
			 * 	1- Detect Features in frame1 using shi-tomasi
			 *	2- Use Pyramidal KLT to estimate motion
			 * 	3- Estimate Fundamental Matrix F (RANSAC, 8-Point, 5-Point )
			 *	4- Compute Essential Matrix from F and Calibration Matrix K
			 *	5- Find R and T by Essential matrix decomposition
			*/

			// Detect Feature
			cv::Mat gray_f1, gray_f2;
			cv::cvtColor(frame1_, gray_f1, CV_BGR2GRAY);
			cv::cvtColor(frame1_, gray_f2, CV_BGR2GRAY);

			extractFeatures();

			#ifdef DEBUG
				std::cerr << "roi size: " << detector_roi_.size() << std::endl;
				std::cerr << "Number of Corners Detected: " << corners_.size() << std::endl;
				draw_features_on_image(gray_f1, corners_);
			#endif


			// Calculate Optical Flow
			trackFeatures()

     		#ifdef DEBUG
     			std::cerr << "Tracked " << corners_.size() << " features." << std::endl;
     			draw_flow_arrows(gray_f1, corners_ , tracked_features_);
     		#endif

     		// Estimating Fundamental Matrix
		}

	}
}
}
