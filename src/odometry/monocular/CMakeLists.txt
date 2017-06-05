INCLUDE_DIRECTORIES(${KITTI_TOOLS_INCLUDE_DIR} ${Boost_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR})

add_executable( monOdometry_test test/monOdometry_test.cpp MonOdometry.cpp )
target_link_libraries(monOdometry_test ${OpenCV_LIBS} ${Boost_LIBRARIES})
