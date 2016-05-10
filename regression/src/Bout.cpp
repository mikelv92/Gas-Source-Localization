/*
 * Bout.cpp
 *
 *  Created on: May 9, 2016
 *      Author: mikel
 */

#include "regression/Bout.h"
Bout::Bout() {
	resetSamples();
}

double* Bout::lowPassFilter()
{
	return odorSamples;
}

double* Bout::differentialFilter()
{
	return odorSamples;
}

double* Bout::ewma()
{
	return odorSamples;
}

int Bout::getBoutCount()
{
	double* a = lowPassFilter();
	double* b = differentialFilter();
	double* c = ewma();
	//Read how many 0-1 switches are.
	int bouts = 1;

	resetSamples();
	return bouts;
}

void Bout::resetSamples()
{
	for (int i = 0; i < MAX_NUM_SAMPLES; i++)
		odorSamples[i] = 0;
	sampleIndex = 0;
}

void Bout::addSample(double sample)
{
	if (sampleIndex < MAX_NUM_SAMPLES)
	{
		odorSamples[sampleIndex] = sample;
		sampleIndex++;
	}
	else
	{
		printf("Odor samples array is full\n");
	}
}

