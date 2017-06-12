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



namespace perception{
namespace odometry{

class MonOdometry{
public:

	/**
	 * @brief      { function_description }
	 */
	MonOdometry();


	/**
	 * @brief      Sets the frames.
	 *
	 * @param[in]  f1    T(const cv::Mat& cameraMatrix);he f 1
	 * @param[in]  f2    The f 2
	 */
	void setFrames(cv::Mat f1, cv::Mat f2);


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
	std::vector<cv::Point2f> tracked_features_;

	cv::Mat cameraMatrix_;
	cv::Mat fundamentalMatrix_;
	cv::Mat essentialMatrix_;


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
	 * @brief      { function_description }
	 *
	 * @param[in]  f          { parameter_description }
	 * @param      <unnamed>  { parameter_description }
	 *
	 * @return     { description_of_the_return_value }
	 */
	virtual cv::Mat findEssentialMatrix(const cv::Mat& f, const cv::Mat& cameraMatrix);


};


}
}

#endif