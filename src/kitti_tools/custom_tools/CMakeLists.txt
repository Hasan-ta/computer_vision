include_directories(${Boost_INCLUDE_DIR})
link_directories(${Boost_LIBRARY_DIR})

add_executable( DisplayImage test.cpp )
target_link_libraries( DisplayImage ${OpenCV_LIBS} ${Boost_LIBRARIES})
