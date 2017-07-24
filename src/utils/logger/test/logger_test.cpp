#include "libs/Logger.hpp"

using namespace perception::utils;
int main (int argc, char** argv)
{
	Logger testLogger(std::string("/home/hasan/Desktop/computer_vision_logging.txt"));

	// testLogger.logString("Logging String 1");
	// testLogger.logString("Logging String 2");

	// cv::Mat testMat = cv::Mat::zeros(4,4,CV_64F);
	// testLogger.logCvStruct<cv::Mat>("zeros", testMat);
	// testMat = cv::Mat::eye(10,11,CV_64F);
	// testLogger.logCvStruct<cv::Mat>("identity", testMat);

	// cv::Point testPoint(10,20);
	// testLogger.logCvStruct<cv::Point>("testPoint", testPoint);

	// testLogger.incrementSequenceNumber();

	// cv::Scalar testScalar(255,255,255);
	// testLogger.logCvStruct<cv::Scalar>("testScalar", testScalar);

	// cv::Size testSize(600,800);
	// testLogger.logCvStruct<cv::Size>("testSize", testSize);

	// testLogger.incrementSequenceNumber();
	// testLogger.logCvStruct<cv::Mat>("identity", testMat);

	cv::Mat testZeros = cv::Mat::zeros(4,4,CV_64F);
	testLogger.addObjectToList(std::string("testZeros"), &testZeros, testLogger.CV_MAT);

	cv::Mat testEye = cv::Mat::eye(10,11,CV_64F);
	testLogger.addObjectToList(std::string("testEye"), &testEye, testLogger.CV_MAT);

	cv::Point testPoint(10,20);
	testLogger.addObjectToList(std::string("testPoint"), &testPoint, testLogger.CV_POINT);

	cv::Size testSize(600,800);
	testLogger.addObjectToList(std::string("testSize"), &testSize, testLogger.CV_SIZE);

	cv::Scalar testScalar(1,1,1);
	testLogger.addObjectToList(std::string("testScalar"), &testScalar, testLogger.CV_SCALAR);

	std::string testString("Hello Logger!");
	testLogger.addObjectToList(std::string("testString"), &testString, testLogger.STD_STRING);

	testLogger.logObjectList();

	testEye = cv::Mat::eye(20,21,CV_64F);
	testZeros += cv::Mat::eye(4,4,CV_64F);
	testZeros *= 5.0;
	testScalar = cv::Scalar(255,255,255);
	testPoint.x = 100; testPoint.y = 200;
	testSize = cv::Size(1200,1600);	

	testLogger.logObjectList();

	return 0;
}
