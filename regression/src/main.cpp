#include "ros/ros.h"

#include "regression/Wind.h"
#include "regression/Position.h"
#include "regression/Bout.h"
#include "regression/WindAvg.h"
#include "regression/GaussianRegression.h"
#include "regression/DataHandler.h"

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;


bool moveBase(Position position)
{
	MoveBaseClient moveBaseClient("move_base", true);

	while (!moveBaseClient.waitForServer(ros::Duration(5.0)))
		ROS_INFO("Waiting for the move_base action server to come up");

	move_base_msgs::MoveBaseGoal goal;

	goal.target_pose.header.frame_id 	= "map";
	goal.target_pose.header.stamp 		= ros::Time::now();

	goal.target_pose.pose.position.x 	= position.getX();
	goal.target_pose.pose.position.y 	= position.getY();
	goal.target_pose.pose.orientation.z	= sin(position.getOrientation() / 2);
	goal.target_pose.pose.orientation.w	= cos(position.getOrientation() / 2);

	ROS_INFO("Sending goal");
	moveBaseClient.sendGoal(goal);
	moveBaseClient.waitForResult();

	if (moveBaseClient.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
		return true;
	else
		return false;
}

void resetSamples(Bout * bout, WindAvg * windAvg)
{
	bout->resetSamples(S1);
	bout->resetSamples(S2);
	bout->resetSamples(S3);
	bout->resetSamples(S4);
	bout->resetSamples(S5);
	bout->resetSamples(S6);
	windAvg->resetSamples();
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "regression");
	ros::NodeHandle n;

	ros::Rate loop_rate(100);

	Position currentPosition(0, 0);

	Bout bout;
	WindAvg windAvg;
	GMap gmap;
	GaussianRegression gaussianRegression;

	gaussianRegression.setGMap(&gmap);

	DataHandler datahandler(&bout, &windAvg, &gmap);
	resetSamples(&bout, &windAvg);

	ros::Subscriber pid_sub 			= n.subscribe("/pid_node/ppm", 1000, &DataHandler::pid_callback, &datahandler);
	ros::Subscriber e_nose_sub 			= n.subscribe("/e_nose_data", 1000, &DataHandler::e_nose_callback, &datahandler);
	ros::Subscriber wind_speed_sub 		= n.subscribe("/windsonic/wind_speed", 1000, &DataHandler::wind_speed_callback, &datahandler);
	ros::Subscriber wind_direction_sub 	= n.subscribe("/windsonic/wind_direction", 1000, &DataHandler::wind_direction_callback, &datahandler);
	ros::Subscriber costmap_sub			= n.subscribe("/move_base/global_costmap/costmap", 1, &DataHandler::costmap_callback, &datahandler);
	ros::Subscriber costmap_update_sub	= n.subscribe("/move_base/global_costmap/costmap_updates", 10, &DataHandler::costmap_update_callback, &datahandler);
	ros::Subscriber ndt_mcl_sub			= n.subscribe("/ndt_mcl", 1000, &DataHandler::ndt_mcl_callback, &datahandler);

	while (ros::ok())
	{
		if (
				bout.isSignalArrayFull(S1) &&
				bout.isSignalArrayFull(S2) &&
				bout.isSignalArrayFull(S3) &&
				bout.isSignalArrayFull(S4) &&
				bout.isSignalArrayFull(S5) &&
				bout.isSignalArrayFull(S6) &&
				windAvg.isSignalArrayFull() &&
				gmap.isInitialized()
			)
		{
			currentPosition = datahandler.getCurrentPosition();
			currentPosition.round();
			gaussianRegression.setCurrentPosition(currentPosition);

            ROS_INFO("Current position: %.2f %.2f", currentPosition.getX(), currentPosition.getY());
            bout.computeBouts();
            double amplitude = bout.getAmplitudeAverage();

			double windDirection = windAvg.getDirectionAverage();
			Wind w = Wind(windAvg.getSpeedAverage(), -windDirection + currentPosition.getOrientation());

			KernelFunction kernelFunction(w);
			gaussianRegression.setKernel(&kernelFunction);
			gaussianRegression.addMeasurement(currentPosition, amplitude);

			Position newPosition;
			do
			{
				newPosition = gaussianRegression.nextBestPosition();

				if (newPosition.isFinalPos())
				{
					ROS_INFO("FINAL POSITION: %.2f %.2f", newPosition.getX(), newPosition.getY());
					ros::shutdown();
				}

				gmap.addTriedPosition(newPosition);
	            ROS_INFO("Next best position: %.2f %.2f", newPosition.getX(), newPosition.getY());
				resetSamples(&bout, &windAvg);
			} while (!moveBase(newPosition));

			ROS_INFO("Base moved.");
			gaussianRegression.updateAlpha();
		}

		ros::spinOnce();
		loop_rate.sleep();
	}
}
