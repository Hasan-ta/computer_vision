#include "MonOdometry.h"

#define DEBUG


cv::Mat rot2euler(const cv::Mat & rotationMatrix)
{
  cv::Mat euler(3,1,CV_64F);

  double m00 = rotationMatrix.at<double>(0,0);
  double m02 = rotationMatrix.at<double>(0,2);
  double m10 = rotationMatrix.at<double>(1,0);
  double m11 = rotationMatrix.at<double>(1,1);
  double m12 = rotationMatrix.at<double>(1,2);
  double m20 = rotationMatrix.at<double>(2,0);
  double m22 = rotationMatrix.at<double>(2,2);

  double x, y, z;

  // Assuming the angles are in radians.
  if (m10 > 0.998) { // singularity at north pole
    x = 0;
    y = CV_PI/2;
    z = atan2(m02,m22);
  }
  else if (m10 < -0.998) { // singularity at south pole
    x = 0;
    y = -CV_PI/2;
    z = atan2(m02,m22);
  }
  else
  {
    x = atan2(-m12,m11);
    y = asin(m10);
    z = atan2(-m20,m00);
  }

  euler.at<double>(0) = x;
  euler.at<double>(1) = y;
  euler.at<double>(2) = z;

  return euler;
}

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

	void MonOdometry::setCameraMatrix(const cv::Mat& cameraMatrix)
	{
		cameraMatrix_ = cameraMatrix.clone();
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
		cv::goodFeaturesToTrack(gray_f1_, corners_, max_corners_, corners_quality_,
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

		cv::calcOpticalFlowPyrLK(gray_f1_, gray_f2_, corners_, tracked_features_, status, err, winSize, 3, termcrit, 0, 0.001);

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

	cv::Mat MonOdometry::findEssentialMatrix(const cv::Mat& f, const cv::Mat& cameraMatrix)
	{
		cv::Mat retMat;
		cv::Mat cameraMatrixTranspose;
		cv::transpose(cameraMatrix, cameraMatrixTranspose);
		retMat = cameraMatrixTranspose * f * cameraMatrix;

		return retMat.clone();
	}

	void MonOdometry::recoverPose(const cv::Mat& e, cv::Mat& R, cv::Mat& t)
	{
		cv::SVD essentialSVD;
		essentialSVD = essentialSVD(e);

		std::cerr << std::endl << "u: " << std::endl << essentialSVD.u << std::endl;
		std::cerr << std::endl << "w: " << std::endl << essentialSVD.w << std::endl; 
		std::cerr << std::endl << "vt: " << std::endl << essentialSVD.vt << std::endl;

		cv::Mat W = cv::Mat::zeros(3,3,CV_64F);
		W.at<double>(0,0) = essentialSVD.w.at<double>(0);
		W.at<double>(1,1) = essentialSVD.w.at<double>(1);
		W.at<double>(2,2) = essentialSVD.w.at<double>(2);

		std::cerr << std::endl << "W: " << std::endl << W << std::endl;

		R = essentialSVD.u*W*essentialSVD.vt;

		std::cerr << std::endl << "R: " << std::endl << R << std::endl;

	    float sy = sqrt(R.at<double>(0,0) * R.at<double>(0,0) +  R.at<double>(1,0) * R.at<double>(1,0) );

        bool singular = sy < 1e-6; // If

        const int RAD2DEG = 180/M_PI;// TODO: Replace by a macro in global header
        
        float r,p,y;
	    cv::Mat measured_eulers(3, 1, CV_64F);
	    measured_eulers = rot2euler(R);
	    r = measured_eulers.at<double>(0,0); p = measured_eulers.at<double>(1,0); y = measured_eulers.at<double>(2,0);
	    std::cerr << "roll: " << r*RAD2DEG << ", pitch: " << p*RAD2DEG << ", yaw: " << y*RAD2DEG << std::endl;

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
			cv::cvtColor(frame1_, gray_f1_, CV_BGR2GRAY);
			cv::cvtColor(frame1_, gray_f2_, CV_BGR2GRAY);

			extractFeatures();

			#ifdef DEBUG
				std::cerr << "roi size: " << detector_roi_.size() << std::endl;
				std::cerr << "Number of Corners Detected: " << corners_.size() << std::endl;
				draw_features_on_image(gray_f1_, corners_);
			#endif


			// Calculate Optical Flow
			trackFeatures();

     		#ifdef DEBUG
     			std::cerr << "Tracked " << corners_.size() << " features." << std::endl;
     			draw_flow_arrows(gray_f1_, corners_ , tracked_features_);
     		#endif

     		// Estimating Fundamental Matrix
     		fundamentalMatrix_ = findFundamentalMat(corners_, tracked_features_);

     		// Estimating Essential Matrix
     		essentialMatrix_ = findEssentialMatrix(fundamentalMatrix_, cameraMatrix_);

     		#ifdef DEBUG
     			std::cerr << std::endl << "essential matrix: " << std::endl << essentialMatrix_ << std::endl;
     		#endif

     		cv::Mat R, t;
     		recoverPose(essentialMatrix_, R, t);
		}

	}
}
}
