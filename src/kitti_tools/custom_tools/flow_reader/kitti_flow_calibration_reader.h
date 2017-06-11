#ifndef KITTI_FLOW_CALIBRATION_READER_H
#define KITTI_FLOW_CALIBRATION_READER_H

#include <fstream>
#include <iostream>
#include <string>
#include <boost/filesystem.hpp>
#include <opencv2/core/core.hpp>
#include "common.h"
#include <vector>


namespace perception{
namespace kitti{

typedef struct CamCalibParams{
	CamCalibParams()
	{
		S = cv::Size(0,0);
		K = cv::Mat::zeros(3,3,CV_64F);
		D = cv::Mat::zeros(1,5,CV_64F);
		P = cv::Mat::zeros(3,4,CV_64F);
		S_rect = cv::Size(0,0);
		R_rect = cv::Mat::zeros(3,3,CV_64F);
		P_rect = cv::Mat::zeros(3,4,CV_64F);
	}

	cv::Size getSize() {return S;}
	cv::Size getSize_rect() {return S_rect;}
	cv::Mat getK() {return K;}
	cv::Mat getD() {return D;}
	cv::Mat getP() {return P;}
	cv::Mat getR_rect() {return R_rect;}
	cv::Mat getP_rect() {return P_rect;}

	cv::Size S, S_rect;
	cv::Mat K, D, P, R_rect, P_rect;
}CamCalibParams;

typedef struct cam2camCalibrationParams{
	std::vector<CamCalibParams> four_cameras_params = std::vector<CamCalibParams>(4,CamCalibParams());
}cam2camCalibrationParams;

class FlowCalibrationReader
{
public:
	FlowCalibrationReader(const std::string& calibration_folder);


	virtual ~FlowCalibrationReader(){}


	void set_calibration_folder(const std::string& path);


	cam2camCalibrationParams get_cam2cam_calibration(const int frameNo, bool train = true);

private:

	void cam2cam_file_reader(const std::string& calFile, cam2camCalibrationParams& params);

	cv::Size sizeParser(std::string& size_values);

	cv::Mat KParser(std::string& K_values);

	cv::Mat DParser(std::string& D_values);

	void RParser(std::string& R_values, cv::Mat& P_mat);

	void TParser(std::string& T_values, cv::Mat& P_mat);

private:
	std::string calibration_folder_;
	std::string training_cal_path_;
	std::string testing_cal_path_;

	bool current_cal_path_valid_ = false;
	bool training_cal_avail_ = false;
	bool testing_cal_avail_ = false;

};
	
}
}

#endif