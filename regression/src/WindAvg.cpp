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
	for (int i = 0; i < Bout::SIGNAL_LEN; i++)
		windSamples[0] = Wind();
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
	float u = 0;
	float v = 0;
	float w = 0;

	int size = Bout::SIGNAL_LEN;
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

