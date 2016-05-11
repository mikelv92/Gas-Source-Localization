/*
 * Bout.h
 *
 *  Created on: May 9, 2016
 *      Author: mikel
 */

#ifndef REGRESSION_SRC_BOUT_H_
#define REGRESSION_SRC_BOUT_H_

#include <stdio.h>
#include <cmath>

#define SMOOTH_STD 5
#define KERNEL_LEN 10 * SMOOTH_STD + 1 //Needs to be odd
#define HALF_LIFE 40
#define DELTA_TIME 1

class Bout {
public:
	const static int MAX_NUM_SAMPLES = 70;

	Bout();
	int getBoutCount();
	void addSample(double sample);
	void resetSamples();
private:
	double signal[MAX_NUM_SAMPLES];
	int sampleIndex;

	float kernelLen;


	void convolute(	const double signal[], const double kernel[], double result[]);
	double gaussianValue(int i);
	void ewma(double signal[], double result[]);

};

#endif /* REGRESSION_SRC_BOUT_H_ */
