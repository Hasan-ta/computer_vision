#include "MonOdometry.h"
#include <iostream>
#include <fstream>

#define DEBUG
static float last_yaw;

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

  return euler.clone();
}

#ifdef DEBUG

void draw_features_on_image(const cv::Mat& gray, const cv::Vector<cv::Point2f>& corners_)
{
	cv::Mat circles_image(gray.rows,gray.cols, CV_8UC3, cv::Scalar(0,0,0));
	for (uint32_t i = 0; i < corners_.size(); ++i)
		cv::circle(circles_image, corners_[i], 3, CV_RGB(0, 255, 0), 2, 2, 0);
	cv::Mat vis;
	// std::cerr << gray.size() << ", " << circles_image.size() << std::endl;
	cv::Mat gray_vis;
	cv::cvtColor(gray, gray_vis, CV_GRAY2BGR);
	addWeighted(gray_vis, 1.0, circles_image, 1.0, 0.0, vis);
	cv::imshow("detected corners", vis);
	cv::waitKey(1);
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

cv::Mat image = cv::Mat::zeros(300,300,CV_64F);

cv::Mat visualizeMotion(const cv::Mat& euler, const cv::Mat& t, const cv::Mat& latestPosition)
{
	const int RAD2DEG = 180/M_PI;// TODO: Replace by a macro in global header
	double yaw = euler.at<double>(2);
	std::cerr << "yaw_pre: " << yaw*RAD2DEG << std::endl;
	if(fabs(euler.at<double>(2)) > (M_PI/2))
		yaw = fabs(fabs(euler.at<double>(2))-M_PI);
	else
		yaw = fabs(fabs(euler.at<double>(2)));
	// std::cerr << "yaw_post: " << yaw*RAD2DEG << std::endl;
	// cv::Mat rot(2,2,CV_64F);
	// rot.at<double>(0,0) = cos(yaw);	rot.at<double>(0,1) = -sin(yaw);
	// rot.at<double>(1,0) = sin(yaw);	rot.at<double>(1,1) = cos(yaw);
	// std::cerr << std::endl << "rot: " << std::endl << rot << std::endl;
	// cv::Mat center(2,1,CV_64F); center.at<double>(0) = 200; center.at<double>(0) = 200;
	// cv::Mat temp = latestPosition;
	// // std::cerr << "late : " << temp.rows << ", " << temp.cols << std::endl;
	// std::cerr << "temp:" << temp << std::endl;
	// cv::Mat newPoint = rot*(latestPosition);
	// // std::cerr << "new point size:" << newPoint.size() << std::endl;
	// std::cerr << std::endl << "new Point: " << newPoint << std::endl;
	// cv::Point p1(200,200); cv::Point p2(newPoint.at<double>(0), newPoint.at<double>(1));

	// cv::Mat image = cv::Mat::zeros(400,400,CV_64F);

	// arrowedLine(image, p1, p2, cv::Scalar(255,0,0), 4);

	cv::Mat trans= (cv::Mat_<double>(2,1) << t.at<double>(0,0), t.at<double>(2,0));
	cv::Mat newPosition(2,1,CV_64F);
	newPosition = latestPosition - (trans);

 	cv::Mat newLineImage = cv::Mat::zeros(300,300,CV_64F);
	cv::Point p1(latestPosition.at<double>(0,0), latestPosition.at<double>(1,0));
	cv::Point p2(newPosition.at<double>(0,0), newPosition.at<double>(1,0));
	std::cerr << "latestPosition: " << p1 << std::endl;
	std::cerr << "newPosition: " << p2 << std::endl;
	std::cerr << "trans: " << trans << std::endl;
	circle(newLineImage, p2, 2, cv::Scalar(255,0,0), 1);
	addWeighted(image, 1.0, newLineImage, 1.0, 0.0, image);
	cv::imshow("result", image);
	cv::waitKey(1);

	return newPosition.clone();
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
	{
		latestPosition_ = cv::Mat(2,1,CV_64F);
		latestPosition_.at<double>(0) = 150;
		latestPosition_.at<double>(1) = 150;
	}

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
	     		continue;
 			}

 			if(sqrtf(powf(corners_[i].x - tracked_features_[i].x,2) + powf(corners_[i].y - tracked_features_[i].y,2)) > 50)
 			{
 				// std::cout << "distacnce : " << sqrtf(powf(corners_[i].x - tracked_features_[i].x,2) + powf(corners_[i].y - tracked_features_[i].y,2)) << std::endl;
 				corners_.erase (corners_.begin() + i - indexCorrection);
	     		tracked_features_.erase (tracked_features_.begin() + i - indexCorrection);
	     		indexCorrection++;
 			}

 		}
	}

	cv::Mat MonOdometry::findEssentialMatrix(const cv::Mat& f, const cv::Mat& cameraMatrix)
	{
		cv::Mat retMat;
		retMat = cameraMatrix.t() * f * cameraMatrix;
		return retMat.clone();
	}

	void MonOdometry::recoverPose(const cv::Mat& e, cv::Mat& R, cv::Mat& t)
	{
		cv::SVD essentialSVD;
		essentialSVD = essentialSVD(e);

		cv::Mat W = cv::Mat::zeros(3,3,CV_64F);

		W.at<double>(0,1) = -1;
		W.at<double>(1,0) = 1;
		W.at<double>(2,2) = 1;

		cv::Mat R1 = (essentialSVD.u)*W.t()*essentialSVD.vt;
		cv::Mat R2 = (essentialSVD.u)*W*essentialSVD.vt;

		t = essentialSVD.u.col(2);

		cv::Mat P0(3,4,R.type()), P1(3,4,R.type()), P2(3,4,R.type()), P3(3,4,R.type()), P4(3,4,R.type());
		P0 = cv::Mat::eye(3,4,CV_64F);
		cv::hconcat(R1, t, P1);
		cv::hconcat(R1, -t, P2);
		cv::hconcat(R2, t, P3);
		cv::hconcat(R2, -t, P4);

		// Triangulate Points
		cv::Mat triangulatedPoints;
		cv::Mat mask1, mask2, mask3, mask4;

		triangulatePoints(P0, P1, corners_, tracked_features_, triangulatedPoints);
		triangulatedPoints.convertTo(triangulatedPoints, R.type());
		mask1 = triangulatedPoints.row(2).mul(triangulatedPoints.row(3))>0;
		triangulatedPoints.row(0) /= triangulatedPoints.row(3);
		triangulatedPoints.row(1) /= triangulatedPoints.row(3);
		triangulatedPoints.row(2) /= triangulatedPoints.row(3);
		triangulatedPoints.row(3) /= triangulatedPoints.row(3);
		mask1 = (triangulatedPoints.row(2) < 30.) & mask1;
		triangulatedPoints = P1 * triangulatedPoints;
		mask1 = (triangulatedPoints.row(2) > 0) & mask1;
		mask1 = (triangulatedPoints.row(2) < 30.) & mask1;

		triangulatePoints(P0, P2, corners_, tracked_features_, triangulatedPoints);
		triangulatedPoints.convertTo(triangulatedPoints, R.type());
		mask2 = triangulatedPoints.row(2).mul(triangulatedPoints.row(3))>0;
		triangulatedPoints.row(0) /= triangulatedPoints.row(3);
		triangulatedPoints.row(1) /= triangulatedPoints.row(3);
		triangulatedPoints.row(2) /= triangulatedPoints.row(3);
		triangulatedPoints.row(3) /= triangulatedPoints.row(3);
		mask2 = (triangulatedPoints.row(2) < 30.) & mask2;
		triangulatedPoints = P1 * triangulatedPoints;
		mask2 = (triangulatedPoints.row(2) > 0) & mask2;
		mask2 = (triangulatedPoints.row(2) < 30.) & mask2;

		triangulatePoints(P0, P3, corners_, tracked_features_, triangulatedPoints);
		triangulatedPoints.convertTo(triangulatedPoints, R.type());
		mask3 = triangulatedPoints.row(2).mul(triangulatedPoints.row(3))>0;
		triangulatedPoints.row(0) /= triangulatedPoints.row(3);
		triangulatedPoints.row(1) /= triangulatedPoints.row(3);
		triangulatedPoints.row(2) /= triangulatedPoints.row(3);
		triangulatedPoints.row(3) /= triangulatedPoints.row(3);
		mask3 = (triangulatedPoints.row(2) < 30.) & mask3;
		triangulatedPoints = P1 * triangulatedPoints;
		mask3 = (triangulatedPoints.row(2) > 0) & mask3;
		mask3 = (triangulatedPoints.row(2) < 30.) & mask3;

		triangulatePoints(P0, P4, corners_, tracked_features_, triangulatedPoints);
		triangulatedPoints.convertTo(triangulatedPoints, R.type());
		mask4 = triangulatedPoints.row(2).mul(triangulatedPoints.row(3))>0;
		triangulatedPoints.row(0) /= triangulatedPoints.row(3);
		triangulatedPoints.row(1) /= triangulatedPoints.row(3);
		triangulatedPoints.row(2) /= triangulatedPoints.row(3);
		triangulatedPoints.row(3) /= triangulatedPoints.row(3);
		mask4 = (triangulatedPoints.row(2) < 30.) & mask4;
		triangulatedPoints = P1 * triangulatedPoints;
		mask4 = (triangulatedPoints.row(2) > 0) & mask4;
		mask4 = (triangulatedPoints.row(2) < 30.) & mask4;


		int nzMask1, nzMask2, nzMask3, nzMask4;
		nzMask1 = cv::countNonZero(mask1);
		nzMask2 = cv::countNonZero(mask2);
		nzMask3 = cv::countNonZero(mask3);
		nzMask4 = cv::countNonZero(mask4);

		if(nzMask1 > nzMask2 > nzMask3 > nzMask4)
		{
			R = R1;
			t = t;
		}
		else if(nzMask2 > nzMask1 > nzMask3 > nzMask4)
		{
			R = R1;
			t = -t;
		}
		else if(nzMask3 > nzMask2 > nzMask1 > nzMask4)
		{
			R = R2;
			t = t;
		}
		else
		{
			R = R2;
			t = -t;
		}

		cv::Mat temp;
		double translation_norm = sqrtf(powf(t.at<double>(0,0),2) + powf(t.at<double>(1,0),2) + powf(t.at<double>(2,0),2));
		// t = t/translation_norm;
		R = R/t.at<double>(2,0);
		cv::hconcat(R,t,temp);

		std::cout << "P: " << std::endl << temp << std::endl;

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
			cv::cvtColor(frame2_, gray_f2_, CV_BGR2GRAY);

			extractFeatures();

			#ifdef DEBUG
				// std::cerr << "roi size: " << detector_roi_.size() << std::endl;
				// std::cerr << "Number of Corners Detected: " << corners_.size() << std::endl;
				draw_features_on_image(gray_f1_, corners_);
			#endif


			// Calculate Optical Flow
			trackFeatures();

     		#ifdef DEBUG
     			// std::cerr << "Tracked " << corners_.size() << " features." << std::endl;
     			draw_flow_arrows(gray_f1_, corners_ , tracked_features_);

     			std::ofstream file("Debug_monodometry.csv", std::ios::out);

     			for (int i =0; i < corners_.size(); ++i)
     			{
     				file << corners_[i].x << "," << corners_[i].y << "," << tracked_features_[i].x << "," << tracked_features_[i].y << "\n";
     			}
     		#endif

     		// Estimating Fundamental Matrix
 		   //  double fx = cameraMatrix_.at<double>(0,0);
		    // double fy = cameraMatrix_.at<double>(1,1);
		    // double cx = cameraMatrix_.at<double>(0,2);
		    // double cy = cameraMatrix_.at<double>(1,2);
     	// 	for (int i =0; i < corners_.size(); ++i)
     	// 	{
     	// 		corners_[i].x = (corners_[i].x - cx) / fx;
     	// 		corners_[i].y = (corners_[i].y - cy) / fy;
     	// 		tracked_features_[i].x = (tracked_features_[i].x - cx) / fx;
     	// 		tracked_features_[i].y = (tracked_features_[i].y - cy) / fy;
 
     	// 	}

     		fundamentalMatrix_ = findFundamentalMat(corners_, tracked_features_);

     		// #ifdef DEBUG
     		// 	std::cerr << std::endl << "fundamental matrix: " << std::endl << fundamentalMatrix_ << std::endl;
     		// #endif

     		// #ifdef DEBUG
     		// 	std::cerr << std::endl << "camera matrix: " << std::endl << cameraMatrix_ << std::endl;
     		// #endif

     		// Estimating Essential Matrix
     		essentialMatrix_ = findEssentialMatrix(fundamentalMatrix_, cameraMatrix_);
     		// essentialMatrix_ = fundamentalMatrix_;

     		// #ifdef DEBUG
     		// 	std::cerr << std::endl << "essential matrix: " << std::endl << essentialMatrix_ << std::endl;
     		// #endif

     		cv::Mat R(3,3,CV_64F), t(3,1,CV_64F), euler;
     		recoverPose(essentialMatrix_, R, t);

     		euler = rot2euler(R);

     		#ifdef DEBUG
     			latestPosition_ = visualizeMotion(euler, t, latestPosition_);
     		#endif	
		}

	}
}
}
