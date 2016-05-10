/*
 * Bout.h
 *
 *  Created on: May 9, 2016
 *      Author: mikel
 */

#ifndef REGRESSION_SRC_BOUT_H_
#define REGRESSION_SRC_BOUT_H_

#include <stdio.h>

class Bout {
public:
	const static int MAX_NUM_SAMPLES = 20;

	Bout();
	int getBoutCount();
	void addSample(double sample);
	void resetSamples();
private:
	double odorSamples[MAX_NUM_SAMPLES];
	int sampleIndex;

	double* lowPassFilter();
	double* differentialFilter();
	double* ewma();

};

#endif /* REGRESSION_SRC_BOUT_H_ */
