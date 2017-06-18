
if (BUILD_ODOMETRY)
	INCLUDE_DIRECTORIES( ${Boost_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR}
				${PARENT_SRC_DIR}
				${EIGEN3_INCLUDE_DIRS})

	add_executable( monOdometry_test test/monOdometry_test.cpp MonOdometry.cpp ${PARENT_SRC_DIR}/kitti_tools/custom_tools/flow_reader/kitti_flow_calibration_reader.cpp)
	target_link_libraries(monOdometry_test ${OpenCV_LIBS} ${Boost_LIBRARIES} )
endif()
