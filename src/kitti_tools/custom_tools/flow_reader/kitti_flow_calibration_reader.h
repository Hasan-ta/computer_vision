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
		ExtMat = cv::Mat::zeros(3,4,CV_64F);
		S_rect = cv::Size(0,0);
		R_rect = cv::Mat::zeros(3,3,CV_64F);
		P_rect = cv::Mat::zeros(3,4,CV_64F);
	}

	cv::Size getSize() {return S;}
	cv::Size getSize_rect() {return S_rect;}
	void getK(cv::Mat& K_ret) {K_ret = K;}
	void getD(cv::Mat& D_ret) {D_ret = D;}
	void getExtMat(cv::Mat& ExtMat_ret) {ExtMat_ret = ExtMat;}
	void getR_rect(cv::Mat& R_rect_ret) {R_rect_ret =  R_rect;}
	void getP_rect(cv::Mat& P_rect_ret) {P_rect_ret = P_rect;}

	cv::Size S, S_rect;
	cv::Mat K, D, ExtMat, R_rect, P_rect;

}CamCalibParams;


typedef struct cam2camCalibrationParams{
	std::vector<CamCalibParams> four_cameras_params = std::vector<CamCalibParams>(4,CamCalibParams());
	void setSize(int camNo, const cv::Size S){four_cameras_params[camNo].S = S;}
	void setSize_rect(int camNo, const cv::Size S_rect){four_cameras_params[camNo].S_rect = S_rect;}
	void setK(int camNo, const cv::Mat& K){four_cameras_params[camNo].K = K.clone();}
	void setD(int camNo, const cv::Mat& D){four_cameras_params[camNo].D = D.clone();}
	void setExtMat(int camNo, const cv::Mat& ExtMat){four_cameras_params[camNo].ExtMat = ExtMat.clone();}
	void setR_rect(int camNo, const cv::Mat& R_rect){four_cameras_params[camNo].R_rect = R_rect.clone();}
	void setP_rect(int camNo, const cv::Mat& P_rect){four_cameras_params[camNo].P_rect = P_rect.clone();}

	void set_camera(int camNo, CamCalibParams& cam)
	{
		setSize(camNo, cam.S);
		setK(camNo,cam.K);
		setD(camNo,cam.D);
		setExtMat(camNo,cam.ExtMat);
		setSize_rect(camNo, cam.S_rect);
		setR_rect(camNo, cam.R_rect);
		setP_rect(camNo, cam.P_rect);
	}

}cam2camCalibrationParams;


/**
 * @brief      Class for kitti flow calibration reader.
 */
class FlowCalibrationReader
{
public:

	/**
	 * @brief      { function_description }
	 *
	 * @param[in]  calibration_folder  The calibration folder
	 */
	FlowCalibrationReader(const std::string& calibration_folder);


	/**
	 * @brief      Destroys the object.
	 */
	virtual ~FlowCalibrationReader(){}


	/**
	 * @brief      Sets the calibration folder.
	 *
	 * @param[in]  path  The path
	 */
	void set_calibration_folder(const std::string& path);


	/**
	 * @brief      Gets the camera 2 camera calibration.
	 *
	 * @param[in]  frameNo  The frame no
	 * @param      param    The parameter
	 * @param[in]  train    The train
	 */
	void get_cam2cam_calibration(const int frameNo, cam2camCalibrationParams& param, bool train = true);

private:

	/**
	 * @brief      { function_description }
	 *
	 * @param[in]  calFile  The cal file
	 * @param      params   The parameters
	 */
	void cam2cam_file_reader(const std::string& calFile, cam2camCalibrationParams& params);


	/**
	 * @brief      { function_description }
	 *
	 * @param      size_values  The size values
	 *
	 * @return     { description_of_the_return_value }
	 */
	cv::Size sizeParser(std::string& size_values);


	/**
	 * @brief      { function_description }
	 *
	 * @param      K_values  The k values
	 * @param      K_mat     The k matrix
	 */


	/**
	 * @brief      { function_description }
	 *
	 * @param      K_values  The k values
	 * @param      K_mat     The k matrix
	 */
	void  KParser(std::string& K_values, cv::Mat& K_mat);


	/**
	 * @brief      { function_description }
	 *
	 * @param      D_values  The d values
	 * @param      D_mat     The d matrix
	 */
	void DParser(std::string& D_values, cv::Mat& D_mat);


	/**
	 * @brief      { function_description }
	 *
	 * @param      R_values  The r values
	 * @param      ExtMat    The extent matrix
	 */
	void RParser(std::string& R_values, cv::Mat& ExtMat);


	/**
	 * @brief      { function_description }
	 *
	 * @param      T_values  The t values
	 * @param      ExtMat    The extent matrix
	 */
	void TParser(std::string& T_values, cv::Mat& ExtMat);


	/**
	 * @brief      { function_description }
	 *
	 * @param      P_values  The p values
	 * @param      P_mat     The p matrix
	 */
	void PParser(std::string& P_values, cv::Mat& P_mat);

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