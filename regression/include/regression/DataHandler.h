/*
 * DataHandler.h
 *
 *  Created on: Jun 2, 2016
 *      Author: mikel
 */

#ifndef REGRESSION_SRC_DATAHANDLER_H_
#define REGRESSION_SRC_DATAHANDLER_H_

#include "regression/Bout.h"

#include "std_msgs/Float32.h"
#include "e_nose/ENoseData.h"

class DataHandler {
private:
	Bout * bout;
public:
	DataHandler(Bout * bout);
	void pid_callback(const std_msgs::Float32::ConstPtr& pid_msg);
	void e_nose_callback(const e_nose::ENoseData::ConstPtr& e_nose_msg);
};

#endif /* REGRESSION_SRC_DATAHANDLER_H_ */
