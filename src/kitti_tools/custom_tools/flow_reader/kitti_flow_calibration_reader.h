#ifndef KITTI_FLOW_CALIBRATION_READER_H
#define KITTI_FLOW_CALIBRATION_READER_H

#include <fstream>
#include <iostream>
#include <string>
#include <boost/filesystem.hpp>


namespace perception{
namespace kitti{

class FlowCalibrationReader
{
public:
	FlowCalibrationReader(const std::string& calibration_folder);


	virtual ~FlowCalibrationReader(){}


	void set_calibration_folder(const std::string& path);


	void get_cam2cam_calibration(const int frameNo, bool train = true);

private:

	void cam2cam_file_reader(const std::string& calFile);

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