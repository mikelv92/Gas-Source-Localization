
cmake_minimum_required(VERSION 2.8.3)
project(simulated_tdlas)

find_package(catkin REQUIRED COMPONENTS 
    roscpp 
    visualization_msgs 
    std_msgs 
    nav_msgs 
    msgs_and_srvs 
    tf
    pcl_ros
)

find_package(PCL REQUIRED)
find_package(Boost REQUIRED)

catkin_package(
	DEPENDS roscpp visualization_msgs std_msgs nav_msgs msgs_and_srvs
)

include_directories(include ${PCL_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

add_executable(simulated_tdlas src/fake_tdlas.cpp)
add_dependencies(simulated_tdlas msgs_and_srvs_generate_messages_cpp)
add_executable(listener_tdlas src/listener.cpp)

target_link_libraries(simulated_tdlas
   ${catkin_LIBRARIES}
   ${Boost_LIBRARIES}
 )

target_link_libraries(listener_tdlas
   ${catkin_LIBRARIES}
   ${Boost_LIBRARIES}
 )


