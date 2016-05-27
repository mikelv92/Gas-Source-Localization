#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/ChannelFloat32.h>
#include <ros/ros.h>
#include <iostream>

using namespace std;

void chatterCallback(const sensor_msgs::PointCloud& msg);
