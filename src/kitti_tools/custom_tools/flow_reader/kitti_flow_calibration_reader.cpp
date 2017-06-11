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

    void FlowCalibrationReader::get_cam2cam_calibration(const int frameNo, cam2camCalibrationParams& param, bool train)
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

            cam2cam_file_reader(filePath, param);
        }
        else
        {
            throw std::runtime_error("Required calibration file unavailable");
        }
    }


    cv::Size FlowCalibrationReader::sizeParser(std::string& size_values)
    {
		std::vector<std::string> values = split(size_values, ' ');
		cv::Size ret((int)stringToNumber<double>(values[1]), (int)stringToNumber<double>(values[2]));
		return ret;
    }

    void FlowCalibrationReader::KParser(std::string& K_values, cv::Mat& K_mat)
    {
		std::vector<std::string> values = split(K_values, ' ');

		uint8_t counter = 1;
		for (uint8_t i = 0; i < 3; ++i)
			for (uint8_t j = 0; j < 3; ++j)
			{
				K_mat.at<double>(i,j) = stringToNumber<double>(values[counter]);
				counter++;
			}
    }

    void FlowCalibrationReader::DParser(std::string& D_values, cv::Mat& D_mat)
    {
		std::vector<std::string> values = split(D_values, ' ');

		uint8_t counter = 1;
		for (uint8_t j = 0; j < 5; ++j)
		{
			D_mat.at<double>(0,j) = stringToNumber<double>(values[counter].c_str());
			counter++;
		}
    }

    void FlowCalibrationReader::RParser(std::string& R_values, cv::Mat& P_mat)
    {
		std::vector<std::string> values = split(R_values, ' ');

		uint8_t counter = 1;
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

		uint8_t counter = 1;
		for (uint8_t j = 0; j < 3; ++j)
		{
			P_mat.at<double>(j,3) = stringToNumber<double>(values[counter]);
			counter++;
		}
    }

    void FlowCalibrationReader::PParser(std::string& P_values, cv::Mat& P_mat)
    {
		std::vector<std::string> values = split(P_values, ' ');

		uint8_t counter = 1;
		for (uint8_t i = 0; i < 3; ++i)
			for (uint8_t j = 0; j < 4; ++j)
			{
				P_mat.at<double>(i,j) = stringToNumber<double>(values[counter]);
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
    		CamCalibParams temp;
			while (std::getline(fs, line))
			{
				std::vector<std::string> label_values = split(line,':');
				std::vector<std::string> labels = split(label_values[0], '_');

				if(labels.size()<=1)
				{
					std::stringstream ss;
					ss << "Invalid calibration file: " << calFile << ", at line: " << lineNo;
					throw std::runtime_error(ss.str());
				}
				else if(labels.size()==2 || labels.size()==3)
				{
					int camIndex;
					if(labels.size() == 2)
						camIndex = (stringToNumber<int>(labels[1]) <= 3) ? stringToNumber<int>(labels[1]): 
								throw std::runtime_error("Error reading calibration file: " + calFile);
					else
						camIndex = (stringToNumber<int>(labels[2]) <= 3) ? stringToNumber<int>(labels[2]): 
								throw std::runtime_error("Error reading calibration file: " + calFile);

					{			

							if(labels[0] == "S")
							{
								if(labels[1] == "rect")
								{
									temp.S_rect = sizeParser(label_values[1]);
									params.setSize_rect(camIndex,temp.S_rect);
								}
								else
								{
									temp.S = sizeParser(label_values[1]);
									params.setSize(camIndex,temp.S);
								}
							}

							else if(labels[0] == "K")
							{
								KParser(label_values[1], temp.K);
								params.setK(camIndex,temp.K);
							}

							else if(labels[0] == "D")
							{
								DParser(label_values[1], temp.D);
								params.setD(camIndex,temp.D);
							}

							else if(labels[0] == "R")
							{
								if(labels[1] == "rect")
								{
									RParser(label_values[1], temp.R_rect);
									params.setR_rect(camIndex,temp.R_rect);
								}
								else
								{
									RParser(label_values[1], temp.P);
									params.setP(camIndex,temp.P);

								}
							}

							else if(labels[0] == "T")
							{
								TParser(label_values[1], temp.P);
								params.setP(camIndex,temp.P);
							}

							else if(labels[0] == "P")
							{
								PParser(label_values[1], temp.P_rect);
								params.setP_rect(camIndex, temp.P_rect);
							}

							else if(labels[0] == "calib" || labels[0] == "corner")
								;

							else
							{
								std::stringstream ss;
								ss << "Invalid calibration file: " << calFile << ", at line: " << lineNo;
								throw std::runtime_error(ss.str());
							}
					}

				}
				lineNo++;
			}
    	}
  	}

}
}