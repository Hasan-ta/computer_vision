#ifndef FLOW_STEREO_IO_H
#define FLOW_STEREO_IO_H

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <string>
#include <exception>
#include <boost/filesystem.hpp>
//#include "custom_exceptions.h"
#include <vector>
#include <algorithm>
#include "kitti_tools/custom_tools/common.h"
#include <map>
#include <sstream>
#include "kitti_flow_calibration_reader.h"
#include "utils/calibration/CamParams.hpp"

namespace perception{
namespace kitti{


    class KittiFlowStereoIO2015{
    public:
        KittiFlowStereoIO2015();

        /*! Constructor. Initiates data paths and lists raw images  
            \param path path to kitti_stereo_flow_data the folder should contain at least a "training" subfolder..  
        */ 
        KittiFlowStereoIO2015(const std::string& path) : parent_directory(path)
        {
            set_data_path(parent_directory);
        }

        KittiFlowStereoIO2015(const std::string& path, const std::string& calib_path) : parent_directory(path),
        calibration_path(calib_path)
        {
            set_data_path(parent_directory);
            cal_reader.set_calibration_folder(calibration_path);
        }

        virtual ~KittiFlowStereoIO2015(){}

        void set_data_path(std::string path)
        {
            using namespace boost::filesystem;

            for (directory_iterator itr(parent_directory); itr!=directory_iterator(); ++itr)
            {
                if(itr->path().filename() == "training" && is_directory(*itr))
                {
                    training_avail = true; current_path_valid = true;
                    training_path = parent_directory + "/training";
                    list_raw_sequences();
                }
                if(itr->path().filename() == "testing" && is_directory(*itr))
                {
                    testing_avail = true;
                    testing_path = parent_directory + "/testing";
                }
            }
            if(!current_path_valid)
                throw std::runtime_error("Invalid Kitti Flow Data Path");
            if(!testing_avail)
                std::cerr << "WARNING: Path only contains training data. Test data won't be read\n";
        }


        void list_raw_sequences()
        {
            using namespace boost::filesystem;

            for (directory_iterator itr(training_path + "/image_2"); itr!=directory_iterator(); ++itr)
            {
                std::string temp(itr->path().filename().string());
                //int a = stringToNumber<int>(temp_vec[0],0);
                raw_sequence.push_back(temp);
            }

            sort(raw_sequence.begin(),raw_sequence.end());
        }

        bool get_next_mono_frames(cv::Mat& frame1, cv::Mat& frame2, bool left=true, bool train = true)
        {
            if(current_position < raw_sequence.size())
            {
                std::string image_path;
                if(train)
                    image_path = training_path;
                else
                    image_path = testing_path;

                if(left)
                {
                    image_path += "/image_2/";
                }
                else
                    image_path += "/image_3/";   

                frame1 = cv::imread(image_path + raw_sequence[current_position++], CV_LOAD_IMAGE_COLOR);
                frame2 = cv::imread(image_path + raw_sequence[current_position++], CV_LOAD_IMAGE_COLOR);
                return true;
            }
            else
            {
                std::cout << "reached end of sequence" << std::endl;
                return false;
            }
        }

        bool get_specific_mono_frames(cv::Mat& frame1, cv::Mat& frame2, int frame_no, bool left=true, bool train = true)
        {
            if(frame_no < raw_sequence.size())
            {
                std::stringstream ss;
                std::stringstream sss;
                ss << frame_no;
                if(ss.str().size()==1)
                    sss << "00000" << frame_no;
                else if (ss.str().size()==2)
                    sss << "0000" << frame_no;
                else
                    sss << "000" << frame_no;

                std::string image_path;
                if(train)
                    image_path = training_path;
                else
                    image_path = testing_path;

                if(left)
                    image_path += "/image_2/";
                else
                    image_path += "/image_3/";   

                frame1 = cv::imread(image_path + sss.str() + "_10.png", CV_LOAD_IMAGE_COLOR);
                frame2 = cv::imread(image_path + sss.str() + "_11.png", CV_LOAD_IMAGE_COLOR);
                return true;
            }
            else
            {
                std::cout << "frame number is larger than the number of sequences" << std::endl;
                return false;
            }

        }

        bool get_next_stereo_frames(cv::Mat& frameL1, cv::Mat& frameL2, cv::Mat& frameR1, cv::Mat& frameR2, bool train = true)
        {
            if(current_position < raw_sequence.size())
            {
                std::string left_path, right_path;
                if(train)
                {
                    left_path = training_path+"/image_2/";
                    right_path = training_path+"/image_3/";
                }
                else
                {
                    left_path = testing_path+"/image_2/";
                    right_path = testing_path+"/image_3/";
                }


                frameL1 = cv::imread(left_path + raw_sequence[current_position], CV_LOAD_IMAGE_COLOR);
                frameR1 = cv::imread(right_path + raw_sequence[current_position++], CV_LOAD_IMAGE_COLOR);
                frameL2 = cv::imread(left_path + raw_sequence[current_position], CV_LOAD_IMAGE_COLOR);
                frameR2 = cv::imread(right_path + raw_sequence[current_position++], CV_LOAD_IMAGE_COLOR);
                return true;
            }
            else
            {
                std::cout << "reached end of sequence" << std::endl;
                return false;
            }
        }

        bool get_specific_stereo_frames(cv::Mat& frameL1, cv::Mat& frameL2, cv::Mat& frameR1, cv::Mat& frameR2, int frame_no, bool train = true)
        {
            if(frame_no < raw_sequence.size())
            {
                std::stringstream ss;
                std::stringstream sss;
                ss << frame_no;
                if(ss.str().size()==1)
                    sss << "00000" << frame_no;
                else if (ss.str().size()==2)
                    sss << "0000" << frame_no;
                else
                    sss << "000" << frame_no;

                std::string left_path, right_path;
                if(train)
                {
                    left_path = training_path+"/image_2/";
                    right_path = training_path+"/image_3/";
                }
                else
                {
                    left_path = testing_path+"/image_2/";
                    right_path = testing_path+"/image_3/";
                }

                frameL1 = cv::imread(left_path + sss.str() + "_10.png", CV_LOAD_IMAGE_COLOR);
                frameR1 = cv::imread(right_path + sss.str() + "_10.png", CV_LOAD_IMAGE_COLOR);
                frameL2 = cv::imread(left_path + sss.str() + "_11.png", CV_LOAD_IMAGE_COLOR);
                frameR2 = cv::imread(right_path + sss.str() + "_11.png", CV_LOAD_IMAGE_COLOR);

                return true;
            }
            else
            {
                std::cerr << "frame number is larger than the number of sequences" << std::endl;
                return false;
            }

        }

        bool get_current_frame_cam2cam_calibration(FlowCamCalibParams& params, bool left = true, bool train = true)
        {
            get_specific_frame_cam2cam_calibration(current_position, params, left, train);
            return true;
        }

        bool get_specific_frame_cam2cam_calibration(int frame_no, FlowCamCalibParams& params, bool left = true, bool train = true)
        {
            cam2camFlowCalibrationParams p_4;
            cal_reader.get_cam2cam_calibration(frame_no, p_4, train);
            int left_right = left? 2 : 3;
            params.set_camera(p_4.four_cameras_params[left_right]);
            return true;
        }

        int get_current_position()
        {
            return current_position;
        }

        void set_current_position(const int& num)
        {
            current_position = num;
        }

    private:
        std::string parent_directory;    
        std::string training_path;
        std::string testing_path;

        bool current_path_valid = false;
        bool training_avail = false;
        bool testing_avail = false;

        int current_position = 0;
        std::vector<std::string> raw_sequence;

        std::string calibration_path;
        FlowCalibrationReader cal_reader;

    };

}

}

#endif