set (FLOW_SOURCE_DIR 	${CMAKE_CURRENT_SOURCE_DIR}/flow_reader)
include_directories(${Boost_INCLUDE_DIR}
	${CMAKE_CURRENT_SOURCE_DIR}
	${FLOW_SOURCE_DIR})

message(STATUS "Hasan Out: " ${CMAKE_CURRENT_SOURCE_DIR})

link_directories(${Boost_LIBRARY_DIR})

#add_executable( DisplayImage test/testflow.cpp )
#target_link_libraries( DisplayImage ${OpenCV_LIBS} ${Boost_LIBRARIES})

add_executable( flow_cal_test test/flow_cal_reader_test.cpp ${FLOW_SOURCE_DIR}/kitti_flow_calibration_reader.cpp)
target_link_libraries( flow_cal_test ${Boost_LIBRARIES} ${OpenCV_LIBS})
