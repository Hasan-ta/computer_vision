#ifndef KITTI_TOOLS_CUSTOM_TOOLS_ODOMETRY_READER_KITTIODOMETRYIO_HPP
#define KITTI_TOOLS_CUSTOM_TOOLS_ODOMETRY_READER_KITTIODOMETRYIO_HPP


#include <opencv2/core/core.hpp>
#include <exception>
#include <boost/filesystem.hpp>
#include <vector>


namespace perception{
namespace kiiti{

class KittiOdometryIO{

public:
	KiitiOdometryIO();


	virtual ~KittiOdometryIO();


	KittiOdometryIO(const std::string&& path) : parentDirectory_(path)


	void setDataPath(const std::string& path);


	void getNextFrame(cv::Mat& left, cv::Mat& Right, sequenceJumping = false);

private:

	std::string parentDirectory_;
	bool currentPathValid;
	bool endOfCurrentSequence;

	uint8_t currentSequence;
	uint8_t currentFrame;

	std::vector<std::string> availableSequences;
	std::vector<std::string> availableFrames;

	void listAvailableSequences(std::string datasetPath);
	void listAvailableFrames(std::string sequencePath);

};

}
}

#endif