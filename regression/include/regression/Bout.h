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

#define SMOOTH_STD 22
#define KERNEL_LEN (int)2 * SMOOTH_STD // 10 * ...
#define HALF_LIFE 29
#define DELTA_TIME 1
#define BOUT_AMP_THRESHOLD 0
#define BOUT_DURATION_THRESHOLD 0

enum SignalIndex { PID, S1, S2, S3, S4, S5, S6, NUM_SIGNALS};

class Bout {
public:
	const static int SIGNAL_LEN = 1000; //10000

	Bout();
	void computeBouts();
	int getBoutCount(SignalIndex signalIndex);
	int getBoutCountAverage();
	double getAmplitude(SignalIndex signalIndex);
	double getAmplitudeAverage();
	void addSample(SignalIndex signalIndex, double sample);
	void resetSamples(SignalIndex signalIndex);
	bool isSignalArrayFull(SignalIndex signalIndex);
	~Bout();
private:
	map<SignalIndex, double *> signalsMap;
	map<SignalIndex, int> boutsMap;
	map<SignalIndex, double> amplitudeMap;

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
