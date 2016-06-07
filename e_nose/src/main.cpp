#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>
#include <iostream>
#include <string>
#include <vector>

#include "e_nose/ENoseData.h"
#include "TermComm.h"
#include "ConfigHandler.h"


using namespace std;

e_nose::ENoseData buildMessage(std::string line)
{
	vector<string> tokens;
	// Skip delimiters at beginning.
	std::string::size_type lastPos = line.find_first_not_of(";", 0);
	// Find first "non-delimiter".
	std::string::size_type pos     = line.find_first_of(";", lastPos);

	while (std::string::npos != pos || std::string::npos != lastPos)
	{
		// Found a token, add it to the vector.
		tokens.push_back(line.substr(lastPos, pos - lastPos));
		// Skip delimiters.  Note the "not_of"
		lastPos = line.find_first_not_of(";", pos);
		// Find next "non-delimiter"
		pos = line.find_first_of(";", lastPos);
	}

	e_nose::ENoseData msg;
	msg.timestamp = atoi(tokens[0].c_str());
	msg.s1 = atoi(tokens[1].c_str());
	msg.s2 = atoi(tokens[2].c_str());
	msg.s3 = atoi(tokens[3].c_str());
	msg.s4 = atoi(tokens[4].c_str());
	msg.s5 = atoi(tokens[5].c_str());
	msg.s6 = atoi(tokens[6].c_str());
	msg.rload_1 = atoi(tokens[7].c_str());
	msg.rload_2 = atoi(tokens[8].c_str());
	msg.rload_3 = atoi(tokens[9].c_str());
	msg.rload_4 = atoi(tokens[10].c_str());
	msg.rload_5 = atoi(tokens[11].c_str());
	msg.rload_6 = atoi(tokens[12].c_str());
	msg.voltage = atoi(tokens[13].c_str());
	return msg;
}


int main(int argc, char **argv)
{

	const char* serialFilename = argc == 2 ? argv[1] : "/dev/ttyACM0";

	TermComm comm = TermComm(serialFilename);

	ConfigHandler configHandler = ConfigHandler(&comm);

	ros::init(argc, argv, "eNose");

	ros::NodeHandle n;

	ros::Publisher eNosePub = n.advertise<e_nose::ENoseData>("e_nose_data", 1000);

	ros::ServiceServer rloadConfigService = n.advertiseService("e_nose_rload_config", &ConfigHandler::rloadConfig, &configHandler);
	ros::ServiceServer voltageConfigService = n.advertiseService("e_nose_voltage_config", &ConfigHandler::voltageConfig, &configHandler);


	ros::Rate loop_rate(100);

	while (ros::ok())
	{
		if (comm.readData() == 0)
		{
			for (int i = 0; i < comm.getLineCount(); i++)
			{
				std::string line = comm.getLine();
				e_nose::ENoseData eNoseDataMsg = buildMessage(line);
				eNosePub.publish(eNoseDataMsg);
			}
		}

		ros::spinOnce();
		loop_rate.sleep();

	}
	return 0;
}
