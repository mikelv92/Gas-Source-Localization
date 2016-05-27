cmake_minimum_required(VERSION 2.8.3)
project(environment)
# Load catkin and all dependencies required for this package
# TODO: remove all from COMPONENTS that are not catkin packages.
find_package(catkin REQUIRED COMPONENTS visualization_msgs msgs_and_srvs tf roscpp)
catkin_package(
	DEPENDS visualization_msgs msgs_and_srvs tf roscpp
)
include_directories(include)
add_executable(environment src/environment.cpp)

target_link_libraries(environment
   ${catkin_LIBRARIES}
 )
