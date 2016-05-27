
cmake_minimum_required(VERSION 2.8.3)
project(msgs_and_srvs)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  std_msgs
  message_generation
)

add_message_files(
  FILES
  ChemicalValue.msg
#  odor.msg
  readings.msg
  SourceLoc.msg
)
#uncomment if you have defined services
add_service_files(
  FILES
  SensorPosition.srv
  SensorPosition2.srv
)


include_directories(${Boost_INCLUDE_DIRS})
generate_messages(
    DEPENDENCIES
    std_msgs
)

catkin_package()
