include_directories(${Boost_INCLUDE_DIR}
	${CMAKE_CURRENT_SOURCE_DIR}
	${PARENT_SRC_DIR}
	${EIGEN3_INCLUDE_DIRS})

message (STATUS "hasan: " ${OpenCV_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})

link_directories(${Boost_LIBRARY_DIR})

#add_executable( DisplayImage test/testflow.cpp ${PARENT_SRC_DIR}/utils/calibration/CamParams.cpp ${FLOW_SOURCE_DIR}/kitti_flow_calibration_reader.cpp)
#target_link_libraries( DisplayImage ${OpenCV_LIBS} ${Boost_LIBRARIES})

#add_executable( flow_cal_test test/flow_cal_reader_test.cpp ${FLOW_SOURCE_DIR}/#kitti_flow_calibration_reader.cpp)
#target_link_libraries( flow_cal_test ${Boost_LIBRARIES} ${OpenCV_LIBS})

add_executable( KittiOdometryIO_test test/KittiOdometryIO_test.cpp libs/KittiOdometryIO.cpp)
target_link_libraries( KittiOdometryIO_test ${Boost_LIBRARIES} ${OpenCV_LIBS})
