/*
 * ENoseConfigHandler.h
 *
 *  Created on: Apr 5, 2016
 *      Author: mikel
 */

#ifndef ENOSECONFIGHANDLER_H_
#define ENOSECONFIGHANDLER_H_
#include "TermComm.h"
#include "e_nose/RLoadConfig.h"
#include "e_nose/VoltageConfig.h"
#include <string>



class ConfigHandler {
private:
	TermComm* comm;
public:
	ConfigHandler(TermComm* comm);
	bool rloadConfig(e_nose::RLoadConfig::Request &req, e_nose::RLoadConfig::Response &res);
	bool voltageConfig(e_nose::VoltageConfig::Request &req, e_nose::VoltageConfig::Response &res);
	virtual ~ConfigHandler();
};

#endif /* ENOSECONFIGHANDLER_H_ */
