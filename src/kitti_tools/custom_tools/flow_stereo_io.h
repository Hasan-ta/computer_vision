#ifndef PNG_READER_H
#define PNG_READER_H

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <string>
#include <dirent.h>
#include <exception>
#include <boost/filesystem.hpp>
#include "custom_exceptions.h"
#include <vector>
#include <algorithm>
#include "common.h"
#include <map>
#include <sstream>


using namespace cv;
using namespace std;


namespace perception{


class KittiFlowStereoIO2015{
public:
    KittiFlowStereoIO2015();

    KittiFlowStereoIO2015(std::string path) : parent_directory(path){
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
            throw invalid_path;
        if(!testing_avail)
            std::cout << "WARNING: Path only contains training data. Test data won't be read\n";
    }

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
            throw invalid_path;
        if(!testing_avail)
            std::cout << "WARNING: Path only contains training data. Test data won't be read\n";
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

    bool get_next_mono_frames(cv::Mat& frame1, cv::Mat& frame2, bool left=true)
    {
        if(current_position < raw_sequence.size())
        {
            std::string image_path(training_path);
            if(left)
            {
                image_path += "/image_2/";
            }
            else
                image_path += "/image_3/";   

            frame1 = imread(image_path + raw_sequence[current_position++], CV_LOAD_IMAGE_COLOR);
            frame2 = imread(image_path + raw_sequence[current_position++], CV_LOAD_IMAGE_COLOR);
            return true;
        }
        else
        {
            std::cout << "reached end of sequence" << std::endl;
            return false;
        }
    }

    bool get_specific_mono_frames(cv::Mat& frame1, cv::Mat& frame2, int frame_no, bool left=true)
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

            std::string image_path(training_path);
            if(left)
            {
                image_path += "/image_2/";
            }
            else
                image_path += "/image_3/";   

            frame1 = imread(image_path + sss.str() + "_10.png", CV_LOAD_IMAGE_COLOR);
            frame2 = imread(image_path + sss.str() + "_11.png", CV_LOAD_IMAGE_COLOR);
            return true;
        }
        else
        {
            std::cout << "frame number is larger than the number of sequences" << std::endl;
            return false;
        }

    }

    bool get_next_stereo_frames(cv::Mat& frameL1, cv::Mat& frameL2, cv::Mat& frameR1, cv::Mat& frameR2)
    {
        if(current_position < raw_sequence.size())
        {
            std::string left_path(training_path+"/image_2/");
            std::string right_path(training_path+"/image_3/");

            frameL1 = imread(left_path + raw_sequence[current_position], CV_LOAD_IMAGE_COLOR);
            frameR1 = imread(right_path + raw_sequence[current_position++], CV_LOAD_IMAGE_COLOR);
            frameL2 = imread(left_path + raw_sequence[current_position], CV_LOAD_IMAGE_COLOR);
            frameR2 = imread(right_path + raw_sequence[current_position++], CV_LOAD_IMAGE_COLOR);
            return true;
        }
        else
        {
            std::cout << "reached end of sequence" << std::endl;
            return false;
        }
    }

    bool get_specific_mono_frames(cv::Mat& frameL1, cv::Mat& frameL2, cv::Mat& frameR1, cv::Mat& frameR2, int frame_no)
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

            std::string left_path(training_path+"/image_2/");
            std::string right_path(training_path+"/image_3/");

            frameL1 = imread(left_path + sss.str() + "_10.png", CV_LOAD_IMAGE_COLOR);
            frameR1 = imread(right_path + sss.str() + "_10.png", CV_LOAD_IMAGE_COLOR);
            frameL2 = imread(left_path + sss.str() + "_11.png", CV_LOAD_IMAGE_COLOR);
            frameR2 = imread(right_path + sss.str() + "_11.png", CV_LOAD_IMAGE_COLOR);

            return true;
        }
        else
        {
            std::cout << "frame number is larger than the number of sequences" << std::endl;
            return false;
        }

    }

    int get_current_position()
    {
        return current_position;
    }

    void set_current_position(const int& num)
    {
        current_position = num;
    }

    ~KittiFlowStereoIO2015(){}

private:
    std::string parent_directory;
    std::string training_path;
    std::string testing_path;
    bool current_path_valid = false;
    bool training_avail = false;
    bool testing_avail = false;
    int current_position = 0;
    std::vector<std::string> raw_sequence;

};

}

#endif