cmake_minimum_required(VERSION 2.8.3)
project(pin_picking)

find_package(catkin REQUIRED COMPONENTS roscpp roslib uv_control cwru_davinci_white_tracker
	#roslint cwru_davinci_kinematics cwru_davinci_control
)

#TODO Is this necessary?
find_package(Boost REQUIRED COMPONENTS system thread)

include_directories(${catkin_INCLUDE_DIRS})

catkin_package(
	INCLUDE_DIRS ${catkin_INCLUDE_DIRS}
	#LIBRARIES
	CATKIN_DEPENDS roscpp roslib uv_control cwru_davinci_white_tracker#roslint 
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

include_directories(
	#include
	${catkin_INCLUDE_DIRS}
	${Boost_INCLUDE_DIR}
)

add_executable(pick_pins src/pick_pins.cpp)
target_link_libraries(pick_pins ${catkin_LIBRARIES} ${Boost_LIBRARIES})
