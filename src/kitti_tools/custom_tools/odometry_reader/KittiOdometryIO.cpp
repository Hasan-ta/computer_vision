#include "KittiOdometryIO.hpp"

namespace perception{
namespace kitti{


KittiOdometrIO::KiitiOdometryIO() :
	currentPathValid(false),
	endOfCurrentSequence(false),
	currentSequence(0),
	currentFrame(0)
	{}

KittiOdometryIO::KittiOdometryIO(const std::string&& path) : 
	parentDirectory_(path),
	currentPathValid(false),
	endOfCurrentSequence(false),
	currentSequence(0),
	currentFrame(0)
{
	setDataPath(parentDirectory_);
}

KittiOdometryIO::setDataPath(const std::string& path)
{
	using namespace boost::filesystem;

	for (directory_iterator itr(parentDirectory); itr!=directory_iterator(); ++itr)
	{
	    if(itr->path().filename() == "sequences" && is_directory(*itr))
	    {
	        currentPathValid = true;
	    }
	    // TODO: Add another level of checking path validity.
	}
	if(!current_path_valid)
	    throw std::runtime_error("Invalid Kitti Flow Data Path");
	else
	{
		listAvailableSequences(parentDirectory_+"/sequences");
	}
}

KittiOdometryIO::listAvailableSequences(std::string datasetPath)
{
	using namespace boost::filesystem;

    for (directory_iterator itr(datasetPath); itr!=directory_iterator(); ++itr)
    {
        std::string temp(itr->path().filename().string());
        availableSequences.push_back(temp);
    }

    sort(availableSequences.begin(),availableSequences.end());
}

KittiOdometryIO::listAvailableFrames(std::string sequencePath)
{
	using namespace boost::filesystem;

    for (directory_iterator itr(sequencePath); itr!=directory_iterator(); ++itr)
    {
        std::string temp(itr->path().filename().string());
        availableSequences.push_back(temp);
    }

    sort(availableFrames.begin(),availableFrames.end());
}

KittiOdometryIO::getNextFrame(cv::Mat& left, cv::Mat& Right, sequenceJumping = false)
{
	if(currentSequence != availableSequences.size())
	{
		if(currentFrame != availableFrames.size())
		{
			currentFrame++;
			cv::imread(cv::set)
		}
		endOfCurrentSequence
		currentSequence++;
	}
}

Kitti

}
}