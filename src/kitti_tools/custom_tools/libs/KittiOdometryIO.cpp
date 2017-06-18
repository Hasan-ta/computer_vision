#include "KittiOdometryIO.hpp"

namespace perception{
namespace kitti{


KittiOdometryIO::KittiOdometryIO() :
	currentPathValid_(false),
	endOfCurrentSequence_(false),
	currentSequence_(0),
	currentFrame_(0)
	{}

KittiOdometryIO::KittiOdometryIO(const std::string&& path) : 
	parentDirectory_(path),
	currentPathValid_(false),
	endOfCurrentSequence_(false),
	currentSequence_(0),
	currentFrame_(0)
{
	setDataPath(parentDirectory_);
}

KittiOdometryIO::~KittiOdometryIO(){}

void KittiOdometryIO::setDataPath(const std::string& path)
{
	using namespace boost::filesystem;

	for (directory_iterator itr(parentDirectory_); itr!=directory_iterator(); ++itr)
	{
	    if(itr->path().filename() == "sequences" && is_directory(*itr))
	    {
	        currentPathValid_ = true;
	    }
	    // TODO: Add another level of checking path validity.
	}
	if(!currentPathValid_)
	    throw std::runtime_error("Invalid Kitti Flow Data Path");
	else
	{
		listAvailableSequences(parentDirectory_+"/sequences");
		listAvailableFrames(parentDirectory_+"/sequences/" + availableSequences_[currentSequence_] + "/image_0/");
	}
}

void KittiOdometryIO::listAvailableSequences(const std::string& datasetPath)
{
	availableSequences_.clear();
	using namespace boost::filesystem;

    for (directory_iterator itr(datasetPath); itr!=directory_iterator(); ++itr)
    {
        std::string temp(itr->path().filename().string());
        availableSequences_.push_back(temp);
    }

    sort(availableSequences_.begin(),availableSequences_.end());
}

void KittiOdometryIO::listAvailableFrames(const std::string& sequencePath)
{
	availableFrames_.clear();
	using namespace boost::filesystem;

    for (directory_iterator itr(sequencePath); itr!=directory_iterator(); ++itr)
    {
        std::string temp(itr->path().filename().string());
        availableFrames_.push_back(temp);
    }

    sort(availableFrames_.begin(),availableFrames_.end());
}

// inline std::string KittiOdometryIO::constructImageName(const int& frameNo)
// {
//     std::stringstream ss;
//     std::stringstream sss;
//     ss << frameNo;
//     if(ss.str().size() == 1)
//         sss << "00000" << frameNo;
//     else if (ss.str().size() == 2)
//         sss << "0000" << frameNo;
//     else if(ss.str().size() == 3)
//         sss << "00" << frameNo;
//     else if(ss.str().size() == 4)
//     	sss << "0" << frameNo;
//     else sss << frameNo;

//     return sss.str() + ".png";
// }

// inline std::string KittiOdometryIO::constructSequenceName(const int& sequenceNo)
// {
// 	std::stringstream ss;
//     std::stringstream sss;
//     ss << sequenceNo;
//     if(ss.str().size() == 1)
//         sss << "0" << sequenceNo;
//     else 
//     	sss << sequenceNo;

//     return sss.str();

// }

void KittiOdometryIO::getNextFrame(cv::Mat& left, cv::Mat& right, bool sequenceJumping)
{
	std::cerr << "currentFrame: " << currentFrame_ << std::endl;
	if(currentSequence_ < availableSequences_.size() && (currentFrame_ < availableFrames_.size()))
	{
		endOfCurrentSequence_ = false;
		std::string currentSequencePath = parentDirectory_ + "/sequences/" + availableSequences_[currentSequence_];
		std::string leftImagePath(currentSequencePath + "/image_0/" + availableFrames_[currentFrame_]);
		std::string rightImagePath(currentSequencePath + "/image_1/" + availableFrames_[currentFrame_]);
		left = cv::imread(leftImagePath);
		right = cv::imread(rightImagePath);
		currentFrame_++;
	}
	
	if(currentFrame_ == (availableFrames_.size()))
	{
		std::cerr << "End of current sequence" << std::endl;
		if(sequenceJumping && (currentSequence_ < availableSequences_.size()-1))
		{
			if(currentSequence_ < availableSequences_.size()-1)
			{
				endOfCurrentSequence_ = true;
				currentSequence_++;
				std::string currentSequencePath = parentDirectory_ + "/sequences/" + availableSequences_[currentSequence_] + 
													"/image_0/";
				listAvailableFrames(currentSequencePath);
				currentFrame_ = 0;
			}
		}
		else
		{
			if(currentSequence_ == availableSequences_.size()-1)
				throw std::runtime_error("End of dataset or sequence jumping is disabled");
		}

	}
}

bool KittiOdometryIO::endOfSequence()
{
	return endOfCurrentSequence_;
}

}
}