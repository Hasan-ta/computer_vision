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

    void FlowCalibrationReader::get_cam2cam_calibration(const int frameNo, bool train)
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

            cam2cam_file_reader(filePath);
        }
        else
        {
            throw std::runtime_error("Required calibration file unavailable");
        }

    }


    void FlowCalibrationReader::cam2cam_file_reader(const std::string& calFile)
    {
    	std::ifstream fs(calFile);

    	if(!fs)
    	{
    		throw std::runtime_error("Couldn't open calibration file: " + calFile);
    	}
    	else
    	{
    		std::string line;
			while (std::getline(fs, line))
			{
    			std::cout << line << std::endl;
			}

    	}
  	}

}
}