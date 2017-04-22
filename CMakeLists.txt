cmake_minimum_required (VERSION 2.8.3)
project(ymbc)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	rospy
	std_msgs
	lib2dl
	)

find_package(OpenCV REQUIRED)

catkin_package(
	INCLUDE_DIRS include
	LIBRARIES ymbc 
	CATKIN_DEPENDS roscpp rospy std_msgs
	DEPENDS system_lib
	)

include_directories(
	include
	${catkin_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
	)

add_library(ymbc
	src/ymbc.cpp)

# key control with keyboard
add_executable(ybkey
	src/ybkey.cpp)
target_link_libraries(ybkey
	${catkin_LIBRARIES}
	ymbc
	ypspur
	)

# kadai1
add_executable(kadai1
	src/kadai1.cpp)
target_link_libraries(kadai1
	${catkin_LIBRARIES}
	ymbc
	ypspur
	)

# kadai2
add_executable(kadai2
	src/kadai2.cpp)
target_link_libraries(kadai2
	${catkin_LIBRARIES}
	${OpenCV_LIBRARIES}
	ymbc
	ypspur
	lib2dl
	)
