/*
 * ENoseConfigHandler.cpp
 *
 *  Created on: Apr 5, 2016
 *      Author: mikel
 */

#include "ConfigHandler.h"

#include <sstream>

using namespace std;
ConfigHandler::ConfigHandler(TermComm* comm)
{
	this->comm = comm;
}

bool ConfigHandler::rloadConfig(e_nose::RLoadConfig::Request &req, e_nose::RLoadConfig::Response &res)
{
	int rload	 	= req.rload_id;
	int value 		= req.value;

	if (value < 2)
	{
		res.result = false;
		res.details = "RLoad value needs to be bigger than 2";
		return true;
	}

	if (rload > 6)
	{
		res.result = false;
		res.details ="RLoad ids are from 1 to 6.";
		return true;
	}

	stringstream ss;
	ss << "RL" << rload << ":" << value << endl;
	string temp = ss.str();
	char const* cmd = temp.c_str();

	if (comm->sendCommand(cmd, strlen(cmd)) == 0)
	{
		res.result = true;
		res.details = "Everything ok.";
		return true;
	}
	else
	{
		res.result = false;
		res.details = "Error in sending command.";
		return false;
	}
}

bool ConfigHandler::voltageConfig(e_nose::VoltageConfig::Request &req, e_nose::VoltageConfig::Response &res)
{
	int value 		= req.value;

	stringstream ss;
	ss << "VHT:" << value << endl;
	string temp = ss.str();
	char const* cmd = temp.c_str();

	if (comm->sendCommand(cmd, strlen(cmd)) == 0)
	{
		res.result = true;
		res.details = "Everything ok.";
		return true;
	}
	else
	{
		res.result = false;
		res.details = "Error in sending command.";
		return false;
	}
}



ConfigHandler::~ConfigHandler() {
	// TODO Auto-generated destructor stub
}

