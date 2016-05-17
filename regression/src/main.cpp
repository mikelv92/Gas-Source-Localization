#include "ros/ros.h"

#include "regression/Wind.h"
#include "regression/KernelFunction.h"
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

	Position currentPosition(6, 10, 2);
	Wind currentWind(0, 0, 0);
	Bout bout;
	WindAvg windAvg;

	int snapshot = 50;
	int sampleCount = 0;
	while (ros::ok())
	{
		if (sampleCount < Bout::SIGNAL_LEN)
		{
			msgs_and_srvs::SensorPosition2 odorSrv;
			for (int i = 0; i < 20; i++)
			{
				odorSrv.request.x[i] = currentPosition.getX();
				odorSrv.request.y[i] = currentPosition.getY();
				odorSrv.request.z[i] = currentPosition.getZ();
			}
			if (odorServiceClient.call(odorSrv))
			{
				double odor = (double)odorSrv.response.odor_r[0];
				bout.addSample(odor);
			}


			read_wind::WindData windSrv;
			windSrv.request.snapshot = snapshot;
			windSrv.request.x = currentPosition.getX();
			windSrv.request.y = currentPosition.getY();
			windSrv.request.z = currentPosition.getZ();

			if (windServiceClient.call(windSrv))
			{
				currentWind.setU((double)windSrv.response.u);
				currentWind.setV((double)windSrv.response.v);
				currentWind.setW((double)windSrv.response.w);
				windAvg.addSample(currentWind);
			}

			sampleCount++;
		}
		else
		{
			printf("Current position: %f, %f", currentPosition.getX(), currentPosition.getY());
			sampleCount = 0;
			printf("Bouts: %d\n", bout.getBoutCount());
			KernelFunction kernelFunction(windAvg.getWindAverage());
			GaussianRegression regression(kernelFunction);
			currentPosition.setY(currentPosition.getY() + 1);
		}

		regression::SetSnapshot setSnapshotSrv;

		snapshot++;
		setSnapshotSrv.request.snapshot = snapshot;
		setSnapshotServiceClient.call(setSnapshotSrv);

		ros::spinOnce();
		loop_rate.sleep();

	}
}
