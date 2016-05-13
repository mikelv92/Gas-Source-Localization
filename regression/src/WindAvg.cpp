/*
 * WindAvg.cpp
 *
 *  Created on: May 9, 2016
 *      Author: mikel
 */

#include "regression/WindAvg.h"

WindAvg::WindAvg() {
	resetSamples();
}

void WindAvg::resetSamples()
{
	for (int i = 0; i < Bout::MAX_NUM_SAMPLES; i++)
		windSamples[0] = Wind();
	windIndex = 0;
}

void WindAvg::addSample(Wind sample)
{
	if (windIndex < Bout::MAX_NUM_SAMPLES)
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
	float u = 0;
	float v = 0;
	float w = 0;

	int size = Bout::MAX_NUM_SAMPLES;
	for (int i = 0; i < size; i++)
	{
		u += windSamples[i].getU();
		v += windSamples[i].getV();
		w += windSamples[i].getW();
	}
	resetSamples();
	return Wind(u / size, v / size, w / size);

}

WindAvg::~WindAvg() {
	// TODO Auto-generated destructor stub
}

