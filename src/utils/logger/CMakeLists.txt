include_directories(${Boost_INCLUDE_DIR}
	${CMAKE_CURRENT_SOURCE_DIR}
	${CMAKE_SOURCE_DIR})


link_directories(${Boost_LIBRARY_DIR})

add_executable( logger_test test/logger_test.cpp libs/Logger_imp.hpp)
target_link_libraries( logger_test ${OpenCV_LIBS})
