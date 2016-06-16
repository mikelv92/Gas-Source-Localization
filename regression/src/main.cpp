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

	ros::Rate loop_rate(100);


	Bout bout;
	WindAvg windAvg;
	DataHandler datahandler(&bout, &windAvg);

	ros::Subscriber pid_sub = n.subscribe("/pid_node/ppm", 1000, &DataHandler::pid_callback, &datahandler);
	ros::Subscriber e_nose_sub = n.subscribe("/e_nose_data", 1000, &DataHandler::e_nose_callback, &datahandler);
	ros::Subscriber wind_speed_sub = n.subscribe("/windsonic/wind_speed", 1000, &DataHandler::wind_speed_callback, &datahandler);
	ros::Subscriber wind_direction_sub = n.subscribe("/windsonic/wind_direction", 1000, &DataHandler::wind_direction_callback, &datahandler);
//	while (ros::ok())
//	{
//		ros::spinOnce();
//		loop_rate.sleep();
//	}

	ros::spin();
}
