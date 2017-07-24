#ifndef UTILS_LOGGER_HPP
#define UTILS_LOGGER_HPP

#include <string>
#include <opencv2/core/core.hpp>
#include <fstream>
#include <iostream>
#include <vector>
#include <memory>
#include <utility>


namespace perception{
namespace utils{

class Logger{

public:
	Logger(const std::string& filePath);

	virtual ~Logger();

	void logString(const std::string& entry);

	template<typename T> void logCvStruct(const std::string& structName, const T& cvStruct);

	void addSeparator(char separatorType);

	void incrementSequenceNumber();

	void addObjectToList(const std::string& objectName, void* object, const int typeId);

	void logObjectList();

private:
	std::string filePath_;
	std::fstream fileHandle_;

	bool fileOpen_ = false;

	int sequenceNumber_;

	std::vector<std::pair<std::string, std::pair<int, void*>>> objectList_;

public:
	static const int CV_MAT=		0;
	static const int CV_POINT=		1;
	static const int CV_SIZE=		2;
	static const int CV_SCALAR=		3;
	static const int STD_STRING=	4;
	static const int STD_INT=		5;
	static const int STD_DOUBLE=	6;
	static const int STD_FLOAT=		7;

};

#include "Logger_imp.hpp"

} // namespace utils
} // namespave perception
  
#endif