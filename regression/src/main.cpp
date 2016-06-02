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

	Position currentPosition(2.5, 2.5, 3);
	printf("Initial position: %lf %lf\n", currentPosition.getX(), currentPosition.getY());
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

	FILE * boutLogs = fopen("bout_logs", "w");
	if (boutLogs == NULL)
	{
		printf("Error opening bout log file\n");
		exit(1);
	}
	FILE * windLogs = fopen("wind_logs", "w");
	if (windLogs == NULL)
	{
		printf("Error opening wind log file\n");
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

			sampleCount++;
		}
		else
		{

			sampleCount = 0;
			snapshot = 99;

			int bouts = bout.getBoutCount();

			Wind windavg = windAvg.getWindAverage();
			windavg.setU(0);
			windavg.setV(-10);
			windavg.set2DSpeed(10);
			printf("Wind: (%f, %f) speed: %f\n", windavg.getU(), windavg.getV(), windavg.get2DSpeed());

//			if (currentPosition.getX() < 10)
//			{
//				if (currentPosition.getY() < 5)
//				{
//					printf("Bouts: %d\n", bouts);
//					fprintf(f, "Bouts: %d\n", bouts);
//					fprintf(boutLogs, "%d\t", bouts);
//					fprintf(windLogs, "(%lf, %lf)\t", windavg.getU(), windavg.getV());
//					currentPosition.setY(currentPosition.getY() + 0.5);
//
//					printf("New position: %f, %f\n", currentPosition.getX(), currentPosition.getY());
//					fprintf(f, "New position: %f, %f\n", currentPosition.getX(), currentPosition.getY());
//
//				}
//				else
//				{
//					currentPosition.setY(0);
//					currentPosition.setX(currentPosition.getX() + 0.5);
//					fprintf(boutLogs, "\n");
//					fprintf(windLogs, "\n");
//					printf("New position: %f, %f\n", currentPosition.getX(), currentPosition.getY());
//					fprintf(f, "New position: %f, %f\n", currentPosition.getX(), currentPosition.getY());
//
//				}
//			}
//			else
//			{
//				if (f) fclose(f);
//				if (boutLogs) fclose(boutLogs);
//				if (windLogs) fclose(windLogs);
//			}

			KernelFunction kernel(windavg);
			regression.setKernel(&kernel);
			regression.addMeasurement(currentPosition, bouts);
			currentPosition = regression.nextBestPosition();

			regression.writeMeanMap(f);


		}


		ros::spinOnce();
		loop_rate.sleep();

	}

}
