/*
 * WindAvg.h
 *
 *  Created on: May 9, 2016
 *      Author: mikel
 */

#ifndef REGRESSION_SRC_WINDAVG_H_
#define REGRESSION_SRC_WINDAVG_H_
#include "regression/Wind.h"
#include "regression/Bout.h"
#include "regression/Utilities.h"

#include <stdio.h>

#define WIND_R_SIGNAL_LEN 500

class WindAvg {
private:
	double * windSpeedSamplesR;
	double * windDirectionSamplesR;
	Wind * windSamples;
	int windIndex;
	int windSpeedIndexR;
	int windDirectionIndexR;

	FILE * logFile;
public:
	WindAvg();
	void resetSamples();
	Wind getWindAverage();
	void addSample(Wind sample);
	void addSpeedSampleR(double sample);
	void addDirectionSampleR(double sample);
	bool isSignalArrayFull();
	void printR();
	~WindAvg();
};

#endif /* REGRESSION_SRC_WINDAVG_H_ */
