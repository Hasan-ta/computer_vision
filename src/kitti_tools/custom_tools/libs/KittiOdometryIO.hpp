#ifndef KITTI_TOOLS_CUSTOM_TOOLS_ODOMETRY_READER_KITTIODOMETRYIO_HPP
#define KITTI_TOOLS_CUSTOM_TOOLS_ODOMETRY_READER_KITTIODOMETRYIO_HPP


#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <exception>
#include <boost/filesystem.hpp>
#include <vector>
#include <string>
#include <sstream>


namespace perception{
namespace kitti{

class KittiOdometryIO{

public:
	KittiOdometryIO();


	virtual ~KittiOdometryIO();


	KittiOdometryIO(const std::string&& path);


	void setDataPath(const std::string& path);


	void getNextFrame(cv::Mat& left, cv::Mat& right, bool sequenceJumping = false);


	void seek(const int& sequenceNumber, const int& frameNumber);


	bool endOfSequence();

private:

	std::string parentDirectory_;
	bool currentPathValid_;
	bool endOfCurrentSequence_;

	uint8_t currentSequence_;
	uint16_t currentFrame_;

	std::vector<std::string> availableSequences_;
	std::vector<std::string> availableFrames_;

	void listAvailableSequences(const std::string& datasetPath);

	void listAvailableFrames(const std::string& sequencePath);

};

}
}

#endif