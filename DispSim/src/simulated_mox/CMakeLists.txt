# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(simulated_mox)
# Load catkin and all dependencies required for this package
# TODO: remove all from COMPONENTS that are not catkin packages.
find_package(catkin REQUIRED COMPONENTS roscpp visualization_msgs std_msgs nav_msgs msgs_and_srvs tf pcl_ros)

FILE(GLOB_RECURSE MYFILES_CPP "src/*.cpp")

catkin_package(
	DEPENDS roscpp visualization_msgs std_msgs nav_msgs msgs_and_srvs tf
)

include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

add_executable(simulated_mox  ${MYFILES_CPP})
add_dependencies(simulated_mox msgs_and_srvs_generate_messages_cpp)
target_link_libraries(simulated_mox
   ${catkin_LIBRARIES}
 )


target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES} ${catkin_LIBRARIES})

