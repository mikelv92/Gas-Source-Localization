/*
 * WindAvg.cpp
 *
 *  Created on: May 9, 2016
 *      Author: mikel
 */

#include "regression/WindAvg.h"

WindAvg::WindAvg() {
	windIndex = 0;
	resetSamples();
}

void WindAvg::resetSamples()
{
	for (int i = 0; i < Bout::SIGNAL_LEN; i++)
		windSamples[i] = Wind(0, 0, 0);
	windIndex = 0;
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

WindAvg::~WindAvg() {
	if (windSamples)
		free(windSamples);
}

