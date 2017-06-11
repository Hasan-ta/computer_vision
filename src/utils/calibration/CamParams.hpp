#ifndef _CAMPARAMS_HPP
#define _CAMPARAMS_HPP

#include <opencv2/core/core.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <string>
#include <exception>
#include <Eigen/Geometry> 

namespace perception{
namespace utils{


/**
 * @brief      Class for pointcloud and camera fusion.
 */
class CamParams{

public:

	/**
	 * @brief      Default Constructor
	 */
	CamParams(){}


	/**
	 * @brief      Constructor that explicitly accepts all parameters
	 *
	 * @param[in]  cameraExtrensicMat  The Extrensid camera matrix
	 * @param[in]  cameraMat      The camera matrix
	 * @param[in]  distCoeffs     The distance coeffs
	 * @param[in]  imageSize      The image size
	 */
	CamParams(const cv::Mat& cameraExtrensicMat,
		     const cv::Mat& cameraMat, const cv::Mat& distCoeffs,
		     const cv::Size& imageSize);


	/**
	 * @brief      Constructor to read params from yml file
	 * 			   The yml file should have the following structure:
	 * 			   
	 * 			   %YAML:1.0
	 *				CameraExtrinsicMat: !!opencv-matrix
	 *				   rows: 4
	 *				   cols: 4
	 *				   dt: d
	 *				   data: [ 1.0280521218760108e-02, -9.6774193488206850e-01,
	 *				       2.5173370524063932e-01, 9.8156848206585270e+00,
	 *				       -9.8535123598142660e-01, 3.3052229775567876e-02,
	 *				       1.6730359188235661e-01, 3.8906624393303058e-01,
	 *				       -1.7022706198882040e-01, -2.4976608572336939e-01,
	 *				       -9.5322591749756835e-01, 3.1480452980384230e+00, 0., 0., 0., 1. ]
	 *				CameraMat: !!opencv-matrix
	 *				   rows: 3
	 *				   cols: 3
	 *				   dt: d
	 *				   data: [ 2.6104230649154479e+03, 0., 7.5996824228224989e+02, 0.,
	 *				       2.6153431052317010e+03, 5.6587401063227730e+02, 0., 0., 1. ]
	 *				DistCoeff: !!opencv-matrix
	 *				   rows: 1
	 *				   cols: 5
	 *				   dt: d
	 *				   data: [ -2.0465434336224123e-01, -4.6297037268530777e-01,
	 *				       -1.5400655695909570e-03, -1.5086289138147731e-03,
	 *				       2.9870196023232181e+00 ]
	 *				ImageSize: [ 1600, 1200 ]
	 *
	 * @param[in]  yamlFile  The yaml file path
	 */

	CamParams(const std::string& yamlFile);


	/**
	 * @brief      Destroys the object.
	 */
	virtual ~CamParams(){}


	/**
	 * @brief      Sets the lidar to camera transform.
	 *
	 * @param[in]  cameraExtrensicMat  The lidar to camera matrix
	 *
	 * @return     { description_of_the_return_value }
	 */
	bool setExtrensicCamTransform(const cv::Mat& cameraExtrensicMat);


	/**
	 * @brief      Sets the camera matrix.
	 *
	 * @param[in]  cameraMat  The camera matrix
	 *
	 * @return     { description_of_the_return_value }
	 */
	bool setCamMat(const cv::Mat& cameraMat);


	/**
	 * @brief      Sets the camera distortion.
	 *
	 * @param[in]  distCoeffs  The distance coeffs
	 *
	 * @return     { description_of_the_return_value }
	 */
	bool setCamDistortion(const cv::Mat& distCoeffs);


	bool setProjectionMatrix(const cv::Mat& projectionMat);


	/**
	 * @brief      Sets the image size.
	 *
	 * @param[in]  imageSize  The image size
	 *
	 * @return     { description_of_the_return_value }
	 */
	bool setImageSize(const cv::Size& imageSize);


	/**
	 * @brief      Sets the parameters from file.
	 *
	 * @param[in]  yamlFile  The yaml file
	 *
	 * @return     { description_of_the_return_value }
	 */
	bool setParamsFromFile(const std::string& yamlFile);


	/**
	 * @brief      Sets the lidar to camera transform from euler.
	 *
	 * @param[in]  roll   The roll
	 * @param[in]  pitch  The pitch
	 * @param[in]  yaw    The yaw
	 *
	 * @return     { description_of_the_return_value }
	 */
	void setExtrensicCamRotationFromEuler(const double& roll, const double& pitch, const double& yaw);


	/**
	 * @brief      Sets the lidar to camera translation.
	 *
	 * @param[in]  tx    The transmit
	 * @param[in]  ty    { parameter_description }
	 * @param[in]  tz    { parameter_description }
	 */
	void setExtrensicCamTranslation(const double& tx, const double& ty, const double& tz);


	/**
	 * @brief      Gets the lidar to camera transform.
	 *
	 * @param      outMat  The out matrix
	 */
	void getExtrensicMat(cv::Mat& outMat);


	/**
	 * @brief      Gets the camera matrix.
	 *
	 * @param      outMat  The out matrix
	 */
	void getCamMat(cv::Mat& outMat);


	/**
	 * @brief      Gets the camera distortion.
	 *
	 * @param      outMat  The out matrix
	 */
	void getCamDistortion(cv::Mat& outMat);


	/**
	 * @brief      Gets the projection matrix.
	 *
	 * @param      outMat  The out matrix
	 */
	void getProjectionMat(cv::Mat& outMat);


	/**
	 * @brief      Gets the image size.
	 *
	 * @param      outSize  The out size
	 */
	void getImageSize(cv::Size& outSize);


private:
	cv::Mat cameraExtrensicMat_; /*!< Extrensic Camera Transformation Matrix */

	cv::Mat cameraMat_; /*!< Camera Matrix */

	cv::Mat distCoeffs_; /*!< Camera Distortion Coefficients */

	cv::Size imageSize_; /*!< Image Size */

	cv::Mat projectionMat_;

	bool cameraExtrensicMatAvail_ = false, camMatAvail_ = false, 
		 distCoeffsAvail_ = false, imageSizeAvail_ = false;

	bool projectionMatSet = false;

	bool allParamsAvail_ = false;




};
}
}


#endif
