#include "ros/ros.h"

#include "regression/Wind.h"
#include "regression/Position.h"
#include "regression/Bout.h"
#include "regression/WindAvg.h"
#include "regression/GaussianRegression.h"
#include "regression/DataHandler.h"

#include <move_base_msgs/MoveBaseActionGoal.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base::MoveBaseActionGoal> Client;

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
	//		Client client("move_base/goal", false);
	//		client.waitForServer();
	//
	//		move_base_msgs::MoveBaseActionGoal goal;
	//
	//		//Fill goal
	//		goal.
	//
	//		client.waitForResult();
	//		if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
	//			printf("Yay! The dishes are now clean");
	//		printf("Current State: %s\n", client.getState().toString().c_str());
	//
	//
	//
	//		ros::spinOnce();
	//		loop_rate.sleep();
	//	}

	Client client("move_base/goal", true);
	client.waitForServer();

	move_base_msgs::MoveBaseActionGoal goal;

	//Fill goal
	goal.target_pose.header.frame_id = "map";
	goal.target_pose.header.stamp = ros::Time::now();

	goal.target_pose.pose.position.x = 1;
	goal.target_pose.pose.position.y = 1;

	client.sendGoal(goal);
	client.waitForResult();
	if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
		printf("Yay!");
//	printf("Current State: %s\n", client.getState().toString().c_str());


	//	ros::spin();
}
