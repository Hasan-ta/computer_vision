#ifndef UTILS_LOGGER_LOGGER_IMP_HPP
#define UTILS_LOGGER_LOGGER_IMP_HPP

// #include "Logger.hpp"

Logger::Logger(const std::string& filePath):
					filePath_(filePath),
					fileHandle_(filePath_,std::ios::out),
					sequenceNumber_(0)
{
	if(!fileHandle_.is_open())
	{
		std::cerr << "---------------------------------------------------------------" << std::endl;
		std::cerr << "Warning: Logger coudln't open log file. Nothing will be logged" << std::endl;
		std::cerr << "---------------------------------------------------------------" << std::endl;
	}
	else
		fileOpen_ = true;
}

Logger::~Logger()
{
	if(fileOpen_)
		fileHandle_.close();
}

void Logger::logString(const std::string& entry)
{
	if(fileOpen_)
		fileHandle_ << "\n[" << sequenceNumber_ << "]--> " << entry << std::endl;	
}

template <typename T> void Logger::logCvStruct(const std::string& structName, const T& cvStruct)
{
	if(fileOpen_)
		fileHandle_ << "\n[" << sequenceNumber_ << "]--> " << structName << ":\n" << cvStruct << std::endl;
}

void Logger::addSeparator(const char separatorType)
{
	if(fileOpen_)
		fileHandle_ << "\n" << std::string(30, separatorType) << std::endl;
}

void Logger::incrementSequenceNumber()
{
	if(fileOpen_)
	{
		sequenceNumber_++;
		addSeparator('-');
	}
}

void Logger::addObjectToList(const std::string& objectName, void* object, const int typeId)
{
	if(typeId > 7 || typeId < 0)
	{
		std::cerr << "---------------------------------------------------------------" << std::endl;
		std::cerr << "Warning: Logger still doesn't support this type." << std::endl;
		std::cerr << "---------------------------------------------------------------" << std::endl;
	}
	objectList_.push_back(std::pair<std::string, std::pair<int, void*>>(objectName, std::pair<int, void*>(typeId, object)));
}

void Logger::logObjectList()
{
	if(fileOpen_)
	{
		for(int i = 0; i < objectList_.size(); ++i)
		{

			if(objectList_[i].second.first ==  CV_MAT)
			{
				cv::Mat* temp = (cv::Mat*)(objectList_[i].second.second);
				logCvStruct<cv::Mat>(objectList_[i].first, *temp);
			}

			else if(objectList_[i].second.first ==  CV_POINT)
			{
				cv::Point* temp = (cv::Point*)(objectList_[i].second.second);
				logCvStruct<cv::Point>(objectList_[i].first, *temp);
			}

			else if(objectList_[i].second.first ==  CV_SIZE)
			{
				cv::Size* temp = (cv::Size*)(objectList_[i].second.second);
				logCvStruct<cv::Size>(objectList_[i].first, *temp);
			}

			else if(objectList_[i].second.first ==  CV_SCALAR)
			{
				cv::Scalar* temp = (cv::Scalar*)(objectList_[i].second.second);
				logCvStruct<cv::Scalar>(objectList_[i].first, *temp);
			}

			else if(objectList_[i].second.first ==  STD_STRING)
			{
				std::string* temp = (std::string*)(objectList_[i].second.second);
				logString(*temp);
			}
		}
		incrementSequenceNumber();	
	}

}


#endif