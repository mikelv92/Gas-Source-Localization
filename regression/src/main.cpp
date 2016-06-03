#include "ros/ros.h"

#include "regression/Wind.h"
#include "regression/Position.h"
#include "regression/Bout.h"
#include "regression/WindAvg.h"
#include "regression/GaussianRegression.h"
#include "regression/DataHandler.h"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "regression");
	ros::NodeHandle n;

	ros::Rate loop_rate(10);


	FILE * logs = fopen("logs", "w");
	if (logs == NULL)
	{
		printf("Error opening log file\n");
		exit(1);
	}

	Bout bout(logs);
	DataHandler datahandler(&bout);

	ros::Subscriber pid_sub = n.subscribe("/pid_node/ppm", 1000, &DataHandler::pid_callback, &datahandler);

	while (ros::ok())
	{
		ros::spinOnce();
		loop_rate.sleep();
	}

}
