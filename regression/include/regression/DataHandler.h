/*
 * DataHandler.h
 *
 *  Created on: Jun 2, 2016
 *      Author: mikel
 */

#ifndef REGRESSION_SRC_DATAHANDLER_H_
#define REGRESSION_SRC_DATAHANDLER_H_

#include "regression/Bout.h"
#include "regression/WindAvg.h"

#include "std_msgs/Float32.h"
#include "e_nose/ENoseData.h"

class DataHandler {
private:
	Bout * bout;
	WindAvg * windAvg;
public:
	DataHandler(Bout * bout, WindAvg * windAvg);
	void pid_callback(const std_msgs::Float32::ConstPtr& pid_msg);
	void e_nose_callback(const e_nose::ENoseData::ConstPtr& e_nose_msg);
	void wind_speed_callback(const std_msgs::Float32::ConstPtr& wind_speed_msg);
	void wind_direction_callback(const std_msgs::Float32::ConstPtr& wind_direction_msg);
};

#endif /* REGRESSION_SRC_DATAHANDLER_H_ */
