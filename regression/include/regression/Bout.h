/*
 * Bout.h
 *
 *  Created on: May 9, 2016
 *      Author: mikel
 */

#ifndef REGRESSION_SRC_BOUT_H_
#define REGRESSION_SRC_BOUT_H_

#include <stdio.h>
#include <stdlib.h>
#include <cmath>
#include <cstring>

#define SMOOTH_STD 5
#define KERNEL_LEN 10 * SMOOTH_STD //Needs to be even
#define HALF_LIFE 40
#define DELTA_TIME 1
#define BOUT_AMP_THRESHOLD 0
#define BOUT_DURATION_THRESHOLD 0

class Bout {
public:
	const static int SIGNAL_LEN = 300;

	Bout(FILE * f);
	int getBoutCount();
	void addSample(double sample);
	void resetSamples();
	~Bout();
private:
	double * signal;
	int sampleIndex;

	float kernelLen;

	FILE * logFile;


	void convolute(double * signal, const double kernel[], int kernelLen);
	void convolute(int * signal, const int kernel[], int kernelLen);
	void populateGaussianFilter(double * kernel);
	void ewma(double * signal);
	void printArray(char const * name, double * array, int len);
	void printArray(char const * name, int * array, int len);

};

#endif /* REGRESSION_SRC_BOUT_H_ */
