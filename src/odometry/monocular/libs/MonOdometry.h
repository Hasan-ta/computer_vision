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
#include <vector>
#include <math.h>


namespace perception{
namespace odometry{



/**
 * @brief      Class for monocular pose estimation
 */
class MonOdometry{
public:

	/**
	 * @brief      Simple Constructor
	 */
	MonOdometry();


	/**
	 * @brief      Destroys the object.
	 */
	virtual ~MonOdometry();

	/**
	 * @brief      Sets the frames.
	 *
	 * @param[in]  f1    T(const cv::Mat& cameraMatrix);he f 1
	 * @param[in]  f2    The f 2
	 */
	void setFrames(cv::Mat f1, cv::Mat f2);

	/**
	 * @brief      Sets the camera matrix.
	 *
	 * @param[in]  cameraMatrix  The camera matrix
	 */
	void setCameraMatrix(const cv::Mat& cameraMatrix);

	/**
	 * @brief      Sets the minimum corner distance.
	 *
	 * @param[in]  distance  The distance
	 */
	void setMinCornerDistance(const int& distance);


	/**
	 * @brief      Calculates the motion.
	 */
	void calculateMotion(cv::Mat& R, cv::Mat& t);


private:
	// Two monocular frames
	cv::Mat frame1_, frame2_;
	cv::Mat gray_f1_, gray_f2_;

	// Feature Detector Params
	cv::vector<cv::Point2f> corners_;
	int max_corners_, min_corner_distance_, detector_block_size_;
	cv::Mat detector_roi_;
	bool use_harris_;
	float corners_quality_;
	std::vector<cv::Point2f> tracked_features_;

	// Camera Params
	cv::Mat cameraMatrix_;


	cv::Mat fundamentalMatrix_;
	cv::Mat essentialMatrix_;

	cv::Mat R_ = cv::Mat::zeros(3,3,CV_64F);
	cv::Mat t_ = cv::Mat::zeros(3,1,CV_64F);

	// Flags
	bool new_frames_ = false;

protected:

	/**
	 * @brief      { function_description }
	 */
	virtual void extractFeatures();


	/**
	 * @brief      { function_description }
	 */
	virtual void trackFeatures();


	/**
	 * @brief      finds essential matrix given fundamental matrix and camera matrix
	 *
	 * @param[in]  f          { parameter_description }
	 * @param      <unnamed>  { parameter_description }
	 *
	 * @return     { description_of_the_return_value }
	 */
	virtual cv::Mat findEssentialMatrix(const cv::Mat& f, const cv::Mat& cameraMatrix);


	/**
	 * @brief      Recovers pose given essential matrix
	 *
	 * @param[in]  e     { parameter_description }
	 * @param      R     { parameter_description }
	 * @param      t     { parameter_description }
	 */
	virtual void recoverPose(const cv::Mat& e, cv::Mat& R, cv::Mat& t);


};


}
}

#endif