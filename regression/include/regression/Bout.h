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
#include <map>

#include "regression/Utilities.h"

using namespace std;

#define SMOOTH_STD 22 //In python it's 30. Check how it works with pandas.rolling_window
#define KERNEL_LEN (int)10 * SMOOTH_STD //Needs to be odd?
#define HALF_LIFE 29
#define DELTA_TIME 1
#define BOUT_AMP_THRESHOLD 0
#define BOUT_DURATION_THRESHOLD 0

enum SignalIndex { PID, S1, S2, S3, S4, S5, S6, NUM_SIGNALS};

class Bout {
public:
	const static int SIGNAL_LEN = 10000;

	Bout();
	int getBoutCount(SignalIndex signalIndex);
	void addSample(SignalIndex signalIndex, double sample);
	void resetSamples(SignalIndex signalIndex);
	bool isSignalArrayFull(SignalIndex signalIndex);
	~Bout();
private:
	map<SignalIndex, double *> signalsMap;

	int sampleIndexes[6];

	float kernelLen;

	FILE * logFile;


	void convolute(double * signal, const double kernel[], int kernelLen);
	void convolute(int * signal, const int kernel[], int kernelLen);
	void populateGaussianFilter(double * kernel);
	void ewma(double * signal);
	double computeAmpThreshold(double * amps, int len);

};

#endif /* REGRESSION_SRC_BOUT_H_ */
