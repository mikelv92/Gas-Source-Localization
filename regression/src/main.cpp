#include "ros/ros.h"

#include "regression/Wind.h"
#include "regression/Position.h"
#include "regression/Bout.h"
#include "regression/WindAvg.h"
#include "regression/GaussianRegression.h"

#include "read_wind/WindData.h"
#include "msgs_and_srvs/SensorPosition2.h"
#include "regression/GetSnapshot.h"
#include "regression/SetSnapshot.h"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "regression");
	ros::NodeHandle n;
	ros::ServiceClient windServiceClient = n.serviceClient<read_wind::WindData>("read_wind");
	ros::ServiceClient odorServiceClient = n.serviceClient<msgs_and_srvs::SensorPosition2>("/odor_value");
	ros::ServiceClient getSnapshotServiceClient = n.serviceClient<regression::GetSnapshot>("getSnapshot");
	ros::ServiceClient setSnapshotServiceClient = n.serviceClient<regression::SetSnapshot>("setSnapshot");

	ros::Rate loop_rate(10);

	int sampleCount = 0;

	FILE * logs = fopen("logs", "w");
	if (logs == NULL)
	{
		printf("Error opening log file\n");
		exit(1);
	}

	FILE * boutLogs = fopen("bout_logs", "w");
	if (boutLogs == NULL)
	{
		printf("Error opening bout log file\n");
		exit(1);
	}

	Bout bout(logs);

	while (ros::ok())
	{
		if (sampleCount < Bout::SIGNAL_LEN)
		{

			bout.addSample(0); //Subscribe
			sampleCount++;
		}
		else
		{

			sampleCount = 0;

			int bouts = bout.getBoutCount();




		}


		ros::spinOnce();
		loop_rate.sleep();

	}

}
