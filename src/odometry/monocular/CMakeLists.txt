if (BUILD_ODOMETRY)
	#find_package( OpenCV 3.1 REQUIRED )
	include_directories(${OpenCV_INCLUDE_DIRS})
	INCLUDE_DIRECTORIES( ${Boost_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR}
				${CMAKE_SOURCE_DIR}
				${EIGEN3_INCLUDE_DIRS})

	add_executable( monOdometry_test test/monOdometry_test.cpp libs/MonOdometry.cpp ${CMAKE_SOURCE_DIR}/kitti_tools/custom_tools/libs/KittiOdometryIO.cpp ${CMAKE_SOURCE_DIR}/utils/logger/libs/Logger_imp.hpp)
	target_link_libraries(monOdometry_test ${OpenCV_LIBS} ${Boost_LIBRARIES} )
endif()
