/*
 * DataHandler.cpp
 *
 *  Created on: Jun 2, 2016
 *      Author: mikel
 */

#include "regression/DataHandler.h"

DataHandler::DataHandler(Bout * bout, WindAvg * windAvg, GMap * gmap) {
	this->bout = bout;
	this->windAvg = windAvg;
	this->gmap = gmap;
	this->robotOrientation = 0;
}


void DataHandler::pid_callback(const std_msgs::Float32::ConstPtr& pid_msg)
{
	if (!bout->isSignalArrayFull(PID)) bout->addSample(PID, (double)pid_msg->data);
	else printf("PID Bouts: %d\n", bout->getBoutCount(PID));
}

void DataHandler::e_nose_callback(const e_nose::ENoseData::ConstPtr& e_nose_msg)
{
	if (!bout->isSignalArrayFull(S1)) bout->addSample(S1, (double)e_nose_msg->s1);
//	else printf("S1 Bouts: %d\n", bout->getBoutCount(S1));

	if (!bout->isSignalArrayFull(S2)) bout->addSample(S2, (double)e_nose_msg->s2);
//	else printf("S2 Bouts: %d\n", bout->getBoutCount(S2));

	if (!bout->isSignalArrayFull(S3)) bout->addSample(S3, (double)e_nose_msg->s3);
//	else printf("S3 Bouts: %d\n", bout->getBoutCount(S3));

	if (!bout->isSignalArrayFull(S4)) bout->addSample(S4, (double)e_nose_msg->s4);
//	else printf("S4 Bouts: %d\n", bout->getBoutCount(S4));

	if (!bout->isSignalArrayFull(S5)) bout->addSample(S5, (double)e_nose_msg->s5);
//	else printf("S5 Bouts: %d\n", bout->getBoutCount(S5));

	if (!bout->isSignalArrayFull(S6)) bout->addSample(S6, (double)e_nose_msg->s6);
//	else printf("S6 Bouts: %d\n", bout->getBoutCount(S6));
}

void DataHandler::wind_speed_callback(const std_msgs::Float32::ConstPtr& wind_speed_msg)
{
	if (!windAvg->isSignalArrayFull()) windAvg->addSpeedSampleR((double)wind_speed_msg->data);
//	else windAvg->printR();
}

void DataHandler::wind_direction_callback(const std_msgs::Float32::ConstPtr& wind_direction_msg)
{
	if (!windAvg->isSignalArrayFull()) windAvg->addDirectionSampleR((double)wind_direction_msg->data);
//	else windAvg->printR();
}

void DataHandler::gmap_callback(const nav_msgs::OccupancyGrid gmap_msg)
{
	unsigned int width = (unsigned int)gmap_msg.info.width;
	unsigned int height = (unsigned int)gmap_msg.info.height;
	double resolution = (double)gmap_msg.info.resolution;
	double origin_x = (double)gmap_msg.info.origin.position.x;
	double origin_y = (double)gmap_msg.info.origin.position.y;

	int * dataArray = (int *)malloc(width * height * sizeof(int));
    for (int i = 0; i < width * height; i++)
            dataArray[i] = (int)gmap_msg.data[i];

	gmap->init(width, height, resolution, origin_x, origin_y, dataArray);

	free(dataArray);
}

void DataHandler::ndt_mcl_callback(const nav_msgs::Odometry ndt_mcl_msg)
{
	double x				= (double)ndt_mcl_msg.pose.pose.position.x;
	double y 				= (double)ndt_mcl_msg.pose.pose.position.y;
	double sin_theta_half = (double)ndt_mcl_msg.pose.pose.orientation.z;
	double cos_theta_half = (double)ndt_mcl_msg.pose.pose.orientation.w;

	currentPosition = Position(x, y);
	currentPosition.setOrientation(2 * atan2(sin_theta_half, cos_theta_half));
}
