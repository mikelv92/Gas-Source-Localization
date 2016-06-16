/*
 * WindAvg.cpp
 *
 *  Created on: May 9, 2016
 *      Author: mikel
 */

#include "regression/WindAvg.h"

WindAvg::WindAvg() {
	windIndex = 0;
	windSpeedIndexR = 0;
	windDirectionIndexR = 0;
	windDirectionSamplesR = (double *)malloc(500 * sizeof(float));
	windSpeedSamplesR = (double *)malloc(500 * sizeof(float));
	resetSamples();

	logFile = fopen("windLogs", "w");
	if (logFile == NULL)
	{
		printf("Error opening wind log file\n");
		exit(1);
	}


}

void WindAvg::resetSamples()
{
	for (int i = 0; i < Bout::SIGNAL_LEN; i++)
		windSamples[i] = Wind(0, 0, 0);
	windIndex = 0;
}

void WindAvg::addSpeedSampleR(float sample)
{
	if (windSpeedIndexR < WIND_R_SIGNAL_LEN)
	{
		windSpeedSamplesR[windSpeedIndexR] = sample;
		windSpeedIndexR++;
	}
	else
	{
		printf("Wind samples array is full\n");
	}

}

void WindAvg::addDirectionSampleR(float sample)
{
	if (windDirectionIndexR < WIND_R_SIGNAL_LEN)
	{
		windDirectionSamplesR[windDirectionIndexR] = sample;
		windDirectionIndexR++;
	}
	else
	{
		printf("Wind samples array is full\n");
	}

}


void WindAvg::addSample(Wind sample)
{
	if (windIndex < Bout::SIGNAL_LEN)
	{
		windSamples[windIndex] = sample;
		windIndex++;
	}
	else
	{
		printf("Wind samples array is full\n");
	}
}

Wind WindAvg::getWindAverage()
{
	double u = 0;
	double v = 0;
	double w = 0;

	int size = Bout::SIGNAL_LEN;
	for (int i = 0; i < size; i++)
	{
		u += windSamples[i].getU();
		v += windSamples[i].getV();
		w += windSamples[i].getW();
	}

	resetSamples();
	Wind windavg(u / size, v / size, w / size);
	return windavg;

}

void WindAvg::printR()
{
	Utilities::printArray(logFile, "Wind Direction", windDirectionSamplesR, WIND_R_SIGNAL_LEN);
	Utilities::printArray(logFile, "Wind speed", windSpeedSamplesR, WIND_R_SIGNAL_LEN);

}
bool WindAvg::isSignalArrayFull()
{
	return windSpeedIndexR >= WIND_R_SIGNAL_LEN || windDirectionIndexR >= WIND_R_SIGNAL_LEN;
}


WindAvg::~WindAvg() {
	if (windSamples)
		free(windSamples);
	if (windSpeedSamplesR)
		free(windSpeedSamplesR);
	if (windDirectionSamplesR)
		free(windDirectionSamplesR);
}

