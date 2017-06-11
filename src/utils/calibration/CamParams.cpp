#include "CamParams.hpp"


namespace perception{
namespace utils{


CamParams::CamParams(const cv::Mat& cameraExtrensicMat,
	     const cv::Mat& cameraMat, const cv::Mat& distCoeffs,
	     const cv::Size& imageSize)
{
	cameraExtrensicMatAvail_ = setExtrensicCamTransform(cameraExtrensicMat);
	if(!cameraExtrensicMatAvail_)
		throw std::runtime_error("Invalid Extrensic Camera Transform");

	camMatAvail_ = setCamMat(cameraMat);
	if(!camMatAvail_)
		throw std::runtime_error("Invalid Camera Matrix");

	distCoeffsAvail_ = setCamDistortion(distCoeffs);
	if(!distCoeffsAvail_)
		throw std::runtime_error("Invalid Camera Distortion Coefficients");		

	imageSizeAvail_ = setImageSize(imageSize);

	allParamsAvail_ = cameraExtrensicMatAvail_ && camMatAvail_ && distCoeffsAvail_
						&& imageSizeAvail_;
}

CamParams::CamParams(const std::string& yamlFile)
{
	if(!setParamsFromFile(yamlFile))
		throw std::runtime_error("Error Reading Camera Params from file: " + yamlFile);
}

bool CamParams::setParamsFromFile(const std::string& yamlFile)
{
	cv::FileStorage fs(yamlFile, cv::FileStorage::READ);

	if (!fs.isOpened())
    {
    	return false;
    }
    else
    {
		fs["cameraExtrensicMat"] >> cameraExtrensicMat_;
		fs["CameraMat"] >> cameraMat_;
		fs["DistCoeff"] >> distCoeffs_;
		fs["ImageSize"] >> imageSize_;


		if((cameraExtrensicMat_.rows == 4) && (cameraExtrensicMat_.cols == 4))
			cameraExtrensicMatAvail_ = true;
		else
			return false;
		if((cameraMat_.rows == 3) && (cameraMat_.cols == 3))
			camMatAvail_ = true;
		else 
			return false;
		if((distCoeffs_.rows == 1) && (distCoeffs_.rows == 5))
			distCoeffsAvail_ = true;
		else 
			return false;
		if((imageSize_.height != 0) && (imageSize_.width != 0))
			imageSizeAvail_ = true;
		else 
			return false;


		allParamsAvail_ = cameraExtrensicMatAvail_ && camMatAvail_ && distCoeffsAvail_
						&& imageSizeAvail_;
		if(allParamsAvail_)
			return true;
		else
			return false;
    }
}

bool CamParams::setExtrensicCamTransform(const cv::Mat& cameraExtrensicMat)
{
	if((cameraExtrensicMat.rows == 4) && (cameraExtrensicMat.cols == 4))
	{
		cameraExtrensicMat_ = cameraExtrensicMat;
		return true;
	}
	else
		return false;
}

bool CamParams::setCamMat(const cv::Mat& cameraMat)
{
	if((cameraMat.rows == 3) && (cameraMat.cols == 3))
	{
		cameraMat_ = cameraMat;
		return true;
	}
	else
		return false;
}

bool CamParams::setCamDistortion(const cv::Mat& distCoeffs)
{
	if((distCoeffs.rows == 1) && (distCoeffs.rows == 5))
	{
		distCoeffs_ = distCoeffs;
		return true;	
	}
	else
		return false;
}

bool CamParams::setProjectionMatrix(const cv::Mat& projectionMat)
{
	if((projectionMat.rows == 3) && (projectionMat.cols==4))
	{
		projectionMat_ =  projectionMat;
		projectionMatSet = true;
		return true;
	}
	else
		return false;
}

bool CamParams::setImageSize(const cv::Size& imageSize)
{
	if((imageSize.height != 0) && (imageSize.width != 0))
	{
		imageSize_ = imageSize;
		return true;		
	}
	else 
		return false;

}

void CamParams::setExtrensicCamRotationFromEuler(const double& roll, const double& pitch, const double& yaw)
{
	Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
	Eigen::AngleAxisd yawAngle(pitch, Eigen::Vector3d::UnitY());
	Eigen::AngleAxisd pitchAngle(yaw, Eigen::Vector3d::UnitZ());

	Eigen::Quaternion<double> q = rollAngle * yawAngle * pitchAngle;

	Eigen::Matrix3d rotationMatrix = q.matrix();

	for (uint8_t i = 0; i < 3; ++i)
		for (uint8_t j = 0; j < 3; ++j)
			cameraExtrensicMat_.at<double>(i,j) = rotationMatrix(i,j);

	cameraExtrensicMat_.at<double>(3,3) = 1;

	cameraExtrensicMatAvail_ = true;
}

void CamParams::setExtrensicCamTranslation(const double& tx, const double& ty, const double& tz)
{
	cameraExtrensicMat_.at<double>(0,3) = tx;
	cameraExtrensicMat_.at<double>(1,3) = ty;
	cameraExtrensicMat_.at<double>(2,3) = tz;

	cameraExtrensicMat_.at<double>(3,4) = 1;

	cameraExtrensicMatAvail_ = true;
}

void CamParams::getExtrensicMat(cv::Mat& outMat)
{
	if(cameraExtrensicMatAvail_)
		outMat = cameraExtrensicMat_;
	else
		throw std::runtime_error("getExtrensicCamTransform: No Extrinsic calibration matrix was set");
}

void CamParams::getCamMat(cv::Mat& outMat)
{
	if(camMatAvail_)
		outMat = cameraMat_;
	else
		throw std::runtime_error("getCamMat: No camera matrix was set");
}

void CamParams::getImageSize(cv::Size& outSize)
{
	if(imageSizeAvail_)
		outSize = imageSize_;
	else
		throw std::runtime_error("getCamMat: Image size was not set");
}

void CamParams::getCamDistortion(cv::Mat& outMat)
{
	if(distCoeffsAvail_)
		outMat = distCoeffs_;
	else
		throw std::runtime_error("getCamMat: Distortion Coefficients were not set");
}

void CamParams::getProjectionMat(cv::Mat& outMat)
{
	if(projectionMatSet)
	{
		outMat = projectionMat_;
	}
	else
	{
		if(cameraExtrensicMatAvail_ && camMatAvail_)
			outMat = cameraMat_ * cameraExtrensicMat_ ;
		else
			throw std::runtime_error("Error returning projection matrix. Set projection matrix or extrinsic camera matrices first");
	}
}

}
}