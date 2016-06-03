/*
 * DataHandler.cpp
 *
 *  Created on: Jun 2, 2016
 *      Author: mikel
 */

#include "regression/DataHandler.h"

DataHandler::DataHandler(Bout * bout) {
	this->bout = bout;
}


void DataHandler::pid_callback(const std_msgs::Float32::ConstPtr& pid_msg)
{
	if (!bout->isSignalArrayFull())
		bout->addSample((double)pid_msg->data);
	else
		printf("Bouts: %d\n", bout->getBoutCount());
}
