#include "kitti_flow_calibration_reader.h"
#include "custom_exceptions.h"
#include <sstream>


namespace perception{
namespace kitti{

	FlowCalibrationReader::FlowCalibrationReader(const std::string& calibration_folder):
	calibration_folder_(calibration_folder)
	{
		set_calibration_folder(calibration_folder_);
	}

	void FlowCalibrationReader::set_calibration_folder(const std::string& path)
    {
        using namespace boost::filesystem;

        for (directory_iterator itr(calibration_folder_); itr!=directory_iterator(); ++itr)
        {
            if(itr->path().filename() == "training" && is_directory(*itr))
            {
                training_cal_avail_ = true; current_cal_path_valid_ = true;
                training_cal_path_ = calibration_folder_ + "/training";
            }
            if(itr->path().filename() == "testing" && is_directory(*itr))
            {
                testing_cal_avail_ = true;
                testing_cal_path_ = calibration_folder_ + "/testing";
            }
        }
        if(!current_cal_path_valid_)
            throw invalid_path;
        if(!testing_cal_avail_)
            std::cerr << "WARNING: Path only contains training calibration data. Test calibration data won't be read\n";
    }

    cam2camCalibrationParams FlowCalibrationReader::get_cam2cam_calibration(const int frameNo, bool train)
    {
        bool cal_avail=(train)? training_cal_avail_ : testing_cal_avail_;
        if(cal_avail)
        {
            std::stringstream ss;
	        std::stringstream sss;
	        ss << frameNo;
	        if(ss.str().size()==1)
	            sss << "00000" << frameNo;
	        else if (ss.str().size()==2)
	            sss << "0000" << frameNo;
	        else
	            sss << "000" << frameNo;

	        std::string filePath;
            if(train)
                filePath = training_cal_path_+"/calib_cam_to_cam/"+sss.str()+".txt";
            else
                filePath = testing_cal_path_+"/calib_cam_to_cam/"+sss.str()+".txt";

            cam2camCalibrationParams returnParams;
            cam2cam_file_reader(filePath, returnParams);
        }
        else
        {
            throw std::runtime_error("Required calibration file unavailable");
        }
    }


    cv::Size FlowCalibrationReader::sizeParser(std::string& size_values)
    {
		std::vector<std::string> values = split(size_values, ' ');
		cv::Size ret(stringToNumber<int>(values[0]), stringToNumber<int>(values[1]));
		return ret;
    }

    cv::Mat FlowCalibrationReader::KParser(std::string& K_values)
    {
		std::vector<std::string> values = split(K_values, ' ');
		cv::Mat ret(3,3,CV_64F);

		uint8_t counter = 0;
		for (uint8_t i = 0; i < 3; ++i)
			for (uint8_t j = 0; j < 3; ++j)
			{
				ret.at<double>(i,j) = stringToNumber<double>(values[counter]);
				counter++;
			}

		return ret;
    }

    cv::Mat FlowCalibrationReader::DParser(std::string& D_values)
    {
		std::vector<std::string> values = split(D_values, ' ');
		cv::Mat ret(1,5,CV_64F);

		uint8_t counter = 0;
		for (uint8_t j = 0; j < 5; ++j)
		{
			ret.at<double>(1,j) = std::atof(values[counter].c_str());
			counter++;
		}

		return ret;
    }

    void FlowCalibrationReader::RParser(std::string& R_values, cv::Mat& P_mat)
    {
		std::vector<std::string> values = split(R_values, ' ');

		uint8_t counter = 0;
		for (uint8_t i = 0; i < 3; ++i)
			for (uint8_t j = 0; j < 3; ++j)
			{
				P_mat.at<double>(i,j) = stringToNumber<double>(values[counter]);
				counter++;
			}
    }

    void FlowCalibrationReader::TParser(std::string& T_values, cv::Mat& P_mat)
    {
		std::vector<std::string> values = split(T_values, ' ');

		uint8_t counter = 0;
		for (uint8_t j = 0; j < 3; ++j)
		{
			P_mat.at<double>(j,3) = stringToNumber<double>(values[counter]);
			counter++;
		}
    }


    void FlowCalibrationReader::cam2cam_file_reader(const std::string& calFile, cam2camCalibrationParams& params)
    {
    	std::ifstream fs(calFile);

    	if(!fs)
    	{
    		throw std::runtime_error("Couldn't open calibration file: " + calFile);
    	}
    	else
    	{
    		std::string line;
    		int lineNo = 0;
			while (std::getline(fs, line))
			{
				std::vector<std::string> label_values = split(line,':');
				std::vector<std::string> labels = split(label_values[0], '_');

				if(labels.size()<=1)
				{
					std::runtime_error("Error Reading Calibration File: " + calFile);
				}
				else if(labels.size()==2)
				{
					int camIndex = (stringToNumber<int>(labels[1]) < 3) ? stringToNumber<int>(labels[1]): 
							throw std::runtime_error("Error reading calibration file: " + calFile);
					if(labels[0] == "S")
						params.four_cameras_params[camIndex].S = sizeParser(label_values[1]);
					else if(labels[0] == "K")
						params.four_cameras_params[camIndex].K = KParser(label_values[1]);
					else if(labels[0] == "D")
						params.four_cameras_params[camIndex].D = DParser(label_values[1]);
					else if(labels[0] == "R")
						RParser(label_values[1], params.four_cameras_params[camIndex].P);
					else if(labels[0] == "T")
						TParser(label_values[1], params.four_cameras_params[camIndex].P);
					else if (labels[0] == "calib" || labels[0] == "corner")
						;
					else
					{
						std::stringstream ss;
						ss << "Invalid calibration file: " << calFile << ", at line: " << lineNo;
						throw std::runtime_error(ss.str());
					}
				}
				else if(labels.size()==3)
				{
					;
				}
				lineNo++;
			}
    	}
  	}

}
}