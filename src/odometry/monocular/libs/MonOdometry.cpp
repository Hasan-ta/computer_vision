#include "MonOdometry.h"

#define DEBUG
#define DEBUG_L2

#ifdef DEBUG

#include <iostream>
#include <fstream>

void draw_features_on_image(const cv::Mat& gray, const cv::vector<cv::Point2f>& corners_)
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

void draw_flow_arrows(const cv::Mat& gray, const cv::vector<cv::Point2f>& pts1, const cv::vector<cv::Point2f>& pts2)
{
	cv::Mat arrows_image(gray.rows,gray.cols, CV_8UC3, cv::Scalar(0,0,0));
	for (uint32_t i = 0; i < pts1.size(); ++i)
		arrowedLine(arrows_image, pts1[i], pts2[i], CV_RGB(255, 0, 0), 3, 8, 0, 0.1);
	cv::Mat vis;	
	cv::Mat gray_vis;
	cv::cvtColor(gray, gray_vis, CV_GRAY2BGR);
	addWeighted(gray_vis, 1.0, arrows_image, 1.0, 0.0, vis);
	cv::imshow("detected corners", vis);
	cv::waitKey(1);
}

std::fstream debugFile("./monOdometry_debug.txt", std::ios::out);

// #include "utils/logger/libs/Logger.hpp"

#endif

namespace perception{
namespace odometry{

	MonOdometry::MonOdometry():
		max_corners_(1000),
		corners_quality_(0.001),
		min_corner_distance_(10),
		detector_block_size_(3),
		use_harris_(true)
		// #ifdef DEBUG 
		// 	,log("/home/hasan/Desktop/MonOdometry_log.txt")
		// #endif
	{
		// #ifdef DEBUG
		// 	log.addObjectToList("fundamentalMatrix_", &fundamentalMatrix_, log.CV_MAT);
		// 	log.addObjectToList("essentialMatrix_", &essentialMatrix_, log.CV_MAT);
		// 	log.addObjectToList("R_", &R_, log.CV_MAT);
		// 	log.addObjectToList("t_", &t_, log.CV_MAT);
		// #endif
	}

	MonOdometry::~MonOdometry()
	{
		#ifdef DEBUG
			debugFile.close();
		#endif
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
		setMinCornerDistance(10);
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
		retMat = cameraMatrix.t() * f * cameraMatrix;

		cv::Mat R(3,3,CV_64F);
		cv::Mat t(3,1,CV_64F);
		cv::SVD essentialSVD;
		essentialSVD = essentialSVD(retMat);

		cv::Mat W = cv::Mat::zeros(3,3,CV_64F);

		W.at<double>(0,1) = -1;
		W.at<double>(1,0) = 1;
		W.at<double>(2,2) = 1;

		cv::Mat R2 = (essentialSVD.u)*W*essentialSVD.vt;

		if(cv::determinant(R2)+1.0 < 1e-9)
		{
			retMat = -1.00 * retMat;
		}

		return retMat.clone();
	}

	void MonOdometry::recoverPose(const cv::Mat& e, cv::Mat& R, cv::Mat& t)
	{
		cv::Mat sigma,u,vt;
		cv::SVD::compute(e,sigma,u,vt);

		cv::Mat W = cv::Mat::zeros(3,3,CV_64F);
		W.at<double>(0,1) = -1;
		W.at<double>(1,0) = 1;
		W.at<double>(2,2) = 1;

		cv::Mat RR(3,3,CV_64F);
		cv::Mat tt(3,1,CV_64F);
		cv::Mat R1 = u*W.t()*vt;
		cv::Mat R2 = u*W*vt;
		tt = u.col(2);

		cv::Mat P0(3,4,R.type()), P1(3,4,R.type()), P2(3,4,R.type()), P3(3,4,R.type()), P4(3,4,R.type());
		P0 = cv::Mat::eye(3,4,CV_64F);
		cv::hconcat(R1, tt, P1);
		cv::hconcat(R1, -tt, P2);
		cv::hconcat(R2, tt, P3);
		cv::hconcat(R2, -tt, P4);

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
		mask1 = (triangulatedPoints.row(2) < 50.) & mask1;
		triangulatedPoints = P1 * triangulatedPoints;
		mask1 = (triangulatedPoints.row(2) > 0) & mask1;
		mask1 = (triangulatedPoints.row(2) < 50.) & mask1;

		triangulatedPoints.release();
		triangulatePoints(P0, P2, corners_, tracked_features_, triangulatedPoints);
		triangulatedPoints.convertTo(triangulatedPoints, R.type());
		mask2 = triangulatedPoints.row(2).mul(triangulatedPoints.row(3))>0;
		triangulatedPoints.row(0) /= triangulatedPoints.row(3);
		triangulatedPoints.row(1) /= triangulatedPoints.row(3);
		triangulatedPoints.row(2) /= triangulatedPoints.row(3);
		triangulatedPoints.row(3) /= triangulatedPoints.row(3);
		mask2 = (triangulatedPoints.row(2) < 50.) & mask2;
		triangulatedPoints = P2 * triangulatedPoints;
		mask2 = (triangulatedPoints.row(2) > 0) & mask2;
		mask2 = (triangulatedPoints.row(2) < 50.) & mask2;

		triangulatedPoints.release();
		triangulatePoints(P0, P3, corners_, tracked_features_, triangulatedPoints);
		triangulatedPoints.convertTo(triangulatedPoints, R.type());
		mask3 = triangulatedPoints.row(2).mul(triangulatedPoints.row(3))>0;
		triangulatedPoints.row(0) /= triangulatedPoints.row(3);
		triangulatedPoints.row(1) /= triangulatedPoints.row(3);
		triangulatedPoints.row(2) /= triangulatedPoints.row(3);
		triangulatedPoints.row(3) /= triangulatedPoints.row(3);
		mask3 = (triangulatedPoints.row(2) < 50.) & mask3;
		triangulatedPoints = P3 * triangulatedPoints;
		mask3 = (triangulatedPoints.row(2) > 0) & mask3;
		mask3 = (triangulatedPoints.row(2) < 50.) & mask3;

		triangulatedPoints.release();
		triangulatePoints(P0, P4, corners_, tracked_features_, triangulatedPoints);
		triangulatedPoints.convertTo(triangulatedPoints, R.type());
		mask4 = triangulatedPoints.row(2).mul(triangulatedPoints.row(3))>0;
		triangulatedPoints.row(0) /= triangulatedPoints.row(3);
		triangulatedPoints.row(1) /= triangulatedPoints.row(3);
		triangulatedPoints.row(2) /= triangulatedPoints.row(3);
		triangulatedPoints.row(3) /= triangulatedPoints.row(3);
		mask4 = (triangulatedPoints.row(2) < 50.) & mask4;
		triangulatedPoints = P4 * triangulatedPoints;
		mask4 = (triangulatedPoints.row(2) > 0) & mask4;
		mask4 = (triangulatedPoints.row(2) < 50.) & mask4;


		int nzMask1, nzMask2, nzMask3, nzMask4;
		nzMask1 = cv::countNonZero(mask1);
		nzMask2 = cv::countNonZero(mask2);
		nzMask3 = cv::countNonZero(mask3);
		nzMask4 = cv::countNonZero(mask4);

		if(nzMask1 > nzMask2 && nzMask1 > nzMask3 && nzMask1 > nzMask4)
		{
			RR = R1;
			tt = tt;
		}
		else if(nzMask2 > nzMask1  && nzMask2> nzMask3  && nzMask2 > nzMask4)
		{
			RR = R1;
			tt = -tt;
		}
		else if(nzMask3 > nzMask2 && nzMask3 > nzMask1 && nzMask3 > nzMask4)
		{
			RR = R2;
			tt = tt;
		}
		else
		{
			RR = R2;
			tt = -tt;
		}

		#ifdef DEBUG
			std::cout << "R1: " << std::endl << R1 << std::endl;
			std::cout << "R2: " << std::endl << R2 << std::endl;
			debugFile << "\n" << "SVD::U \n" << u << "\n";
			debugFile << "\n" << "SVD::Vt \n" << vt << "\n";
			debugFile << "\n" << "R1: \n" << R1 << "\n";
			debugFile << "\n" << "R2: \n" << R2 << "\n";
			debugFile << "\n" << "t: \n" << tt << "\n";
		#endif
		
		R = RR.clone();
		t = tt.clone();
	}

	void MonOdometry::calculateMotion(cv::Mat& R, cv::Mat& t)
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

     // 			double average_x=0.0, average_y=0.0;
     // 			for(int i =0; i < tracked_features_.size(); ++i)
 				// {
 				// 	std::cerr << "corner: [" << corners_[i] << ", tracked: " << tracked_features_[i] << std::endl;
 				// 	average_x += (corners_[i].x-tracked_features_[i].x)/corners_.size();
 				// 	average_y += (corners_[i].y-tracked_features_[i].y)/corners_.size();
 				// }
     // 			std::cerr << "average x motion: " << average_x << ", average y motion: " << average_y << std::endl;
     		#endif

     		// Estimating Fundamental Matrix
 		    double fx = cameraMatrix_.at<double>(0,0);
		    double fy = cameraMatrix_.at<double>(1,1);
		    double cx = cameraMatrix_.at<double>(0,2);
		    double cy = cameraMatrix_.at<double>(1,2);
     		for (int i =0; i < corners_.size(); ++i)
     		{
     			corners_[i].x = (corners_[i].x - cx) / fx;
     			corners_[i].y = (corners_[i].y - cy) / fy;
     			tracked_features_[i].x = (tracked_features_[i].x - cx) / fx;
     			tracked_features_[i].y = (tracked_features_[i].y - cy) / fy;
     		}

     		fundamentalMatrix_ = findFundamentalMat(corners_, tracked_features_, CV_FM_8POINT);

     		#ifdef DEBUG
     			std::cerr << std::endl << "fundamental matrix: " << std::endl << fundamentalMatrix_ << std::endl;
     			debugFile << std::endl << "fundamental matrix: " << std::endl << fundamentalMatrix_ << std::endl;
     		#endif

     		#ifdef DEBUG
     			std::cerr << std::endl << "camera matrix: " << std::endl << cameraMatrix_ << std::endl;
     			// debugFile << std::endl << "camera matrix: " << std::endl << cameraMatrix_ << std::endl;
     		#endif

     		// Estimating Essential Matrix
     		essentialMatrix_ = findEssentialMatrix(fundamentalMatrix_, cameraMatrix_);     		

     		#ifdef DEBUG
     			std::cerr << std::endl << "essential matrix: " << std::endl << essentialMatrix_ << std::endl;
     			debugFile << std::endl << "essential matrix: " << std::endl << essentialMatrix_ << std::endl;
     		#endif

     		#ifdef DEBUG_L2
	 		   // 	double fx = cameraMatrix_.at<double>(0,0);
			    // double fy = cameraMatrix_.at<double>(1,1);
			    // double cx = cameraMatrix_.at<double>(0,2);
			    // double cy = cameraMatrix_.at<double>(1,2);
			    cv::Mat E_cv3;
     			E_cv3 = cv::findEssentialMat(corners_, tracked_features_, fx, cv::Point2d(cx, cy));
     			std::cout << "e: " << std::endl << essentialMatrix_ << std::endl;
     			cv::Mat ff;
     			ff = cameraMatrix_.t().inv() * E_cv3 * cameraMatrix_.inv();
     			std::cout << "f: " << std::endl << fundamentalMatrix_ << std::endl;
     			std::cout << "ff: " << std::endl << ff << std::endl;
     			std::cout << "E_cv3: " << std::endl << E_cv3 << std::endl;

     			for (int i = 0; i < 9; ++i)
     			{
     				if(essentialMatrix_.at<double>(i) != E_cv3.at<double>(i))
     					std::cout << "Non Matching essentials!!!" << std::endl;
     			}

     			
     		#endif 

     		// recoverPose(essentialMatrix_, R_, t_);
     			cv::recoverPose(E_cv3, corners_, tracked_features_, R_, t_, fx, cv::Point2d(cx, cy));

     		#ifdef DEBUG
     			debugFile << std::endl << "R: " << std::endl << R_ << std::endl;
     			debugFile << std::endl << "t: " << std::endl << t_ << std::endl;
     			debugFile << std::string(30, '-') << std::endl;
     		#endif

     		R = R_.clone();
     		t = t_.clone();

		}

	}
}
}
