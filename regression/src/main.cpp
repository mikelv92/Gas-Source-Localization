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

	Position currentPosition(7, 12, 3);
	Wind currentWind(0, 0, 0);
	WindAvg windAvg;

	int snapshot = 99;
	int sampleCount = 0;

	FILE * f = fopen("logs", "w");
	if (f == NULL)
	{
		printf("Error opening log file\n");
		exit(1);
	}

	Bout bout(f);
	GaussianRegression regression;

	while (ros::ok())
	{
		snapshot++;
		regression::SetSnapshot setSnapshotSrv;
		setSnapshotSrv.request.snapshot = snapshot;
		setSnapshotServiceClient.call(setSnapshotSrv);

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

			sampleCount = 0;
			snapshot = 99;

			int bouts = bout.getBoutCount();

			printf("Bouts: %d\n", bouts);
			fprintf(f, "Bouts: %d\n", bouts);

			Wind windavg = windAvg.getWindAverage();
			printf("Wind: (%f, %f) speed: %f\n", windavg.getU(), windavg.getV(), windavg.get2DSpeed());
			KernelFunction kernel(windavg);
			regression.setKernel(&kernel);
			regression.addMeasurement(currentPosition, bouts);
			currentPosition = regression.nextBestPosition();
			printf("New position: %f, %f\n", currentPosition.getX(), currentPosition.getY());
			fprintf(f, "New position: %f, %f\n", currentPosition.getX(), currentPosition.getY());

		}


		ros::spinOnce();
		loop_rate.sleep();

	}
	if (f) fclose(f);

}
