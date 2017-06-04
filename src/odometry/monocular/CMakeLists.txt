INCLUDE_DIRECTORIES(${KITTI_TOOLS_INCLUDE_DIR} ${Boost_INCLUDE_DIRS})

add_executable( monOdometry_test monOdometry_test.cpp MonOdometry.cpp )
target_link_libraries(monOdometry_test ${OpenCV_LIBS} ${Boost_LIBRARIES})
