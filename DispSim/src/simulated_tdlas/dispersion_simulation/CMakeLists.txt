
cmake_minimum_required(VERSION 2.8.3)
project(dispersion_simulation)

find_package(catkin REQUIRED COMPONENTS roscpp visualization_msgs std_msgs nav_msgs msgs_and_srvs pcl_ros)

find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)

catkin_package(
    DEPENDS roscpp visualization_msgs std_msgs nav_msgs msgs_and_srvs
)

include_directories(include)

include_directories(${catkin_INCLUDE_DIRS} $(PCL_INCLUDE_DIRS) ${OpenCV_INCLUDE_DIRS})
MESSAGE("PCL ARE: ${PCL_INCLUDE_DIRS}")

FILE(GLOB_RECURSE MYFILES_C "src/*.c")
FILE(GLOB_RECURSE MYFILES_CPP "src/*.cpp")
add_executable(odorsimulation ${MYFILES_C} ${MYFILES_CPP})

add_dependencies(odorsimulation msgs_and_srvs_generate_msgs_cpp)
target_link_libraries(odorsimulation
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
 )
