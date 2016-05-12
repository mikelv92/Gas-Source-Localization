/*
 * Bout.cpp
 *
 *  Created on: May 9, 2016
 *      Author: mikel
 */

#include "regression/Bout.h"
Bout::Bout() {
	sampleIndex = 0;
	resetSamples();
}

int Bout::getBoutCount()
{
	// Smooth
	double smoothedSignal[MAX_NUM_SAMPLES];
	double gauss_kernel[KERNEL_LEN];
	for (int i = 0; i < KERNEL_LEN; i++)
		gauss_kernel[i] = gaussianValue(i);
	convolute(signal, gauss_kernel, smoothedSignal);

	// Diff
	double diffSignal[MAX_NUM_SAMPLES];
	double diff_kernel[2] = { -1, 1};
	convolute(smoothedSignal, diff_kernel, diffSignal);

	// EWMA
	double ewmaSignal[MAX_NUM_SAMPLES];
	ewma(diffSignal, ewmaSignal);

	// Derivative
	double derivative[MAX_NUM_SAMPLES];
	convolute(ewmaSignal, diff_kernel, derivative);

	// Positive part of the derivative
	for (int i = 0; i < ELEMENT_COUNT(derivative); i++)
		if (derivative[i] >= 0)
			derivative[i] = 1;
		else
			derivative[i] = 0;

	double sign_change[MAX_NUM_SAMPLES];
	convolute(derivative, diff_kernel, sign_change);

	// Get positive and negative indexes
	int* pos_changes = (int*)malloc(MAX_NUM_SAMPLES * sizeof(int));
	int* neg_changes = (int*)malloc(MAX_NUM_SAMPLES * sizeof(int));

	int pos_j = 0, neg_j = 0;
	for (int i = 0; i < ELEMENT_COUNT(sign_change); i++)
	{
		if (sign_change[i] > 0)
			pos_changes[pos_j++] = i;
		if (sign_change[i] < 0)
			neg_changes[neg_j++] = i;
	}

	if (pos_changes[0] > neg_changes[0])
		neg_changes += sizeof(int); //discard first negative change

	int poslen = pos_j + 1; int neglen = neg_j + 1;
	if (poslen > neglen) //lengths must be equal
	{
		int difference = poslen - neglen;
		poslen -= difference;
	}

	int posneg[2][poslen];
	for (int i = 0; i < poslen; i++)
	{
		posneg[0][i] = pos_changes[i];
		posneg[1][i] = neg_changes[i];
	}

	// Get amplitudes
	double amps[poslen];
	for (int i = 0; i < poslen; i++)
		amps[i] = ewmaSignal[posneg[1][i]] - ewmaSignal[posneg[0][i]];

	// Filter amplitudes according to a threshold
	int superThreshAmpIndexes[poslen];
	int thresh_i = 0;
	for (int i = 0; i < poslen; i++)
		if (amps[i] > BOUT_AMP_THRESHOLD)
			superThreshAmpIndexes[thresh_i++] = i;
	int filteredAmpsSize = thresh_i + 1;

	// Get indices of those that passed the threshold
	int filteredAmp_posneg[2][filteredAmpsSize];
	for (int i = 0; i < filteredAmpsSize; i++)
	{
		filteredAmp_posneg[0][i] = posneg[0][superThreshAmpIndexes[i]];
		filteredAmp_posneg[1][i] = posneg[1][superThreshAmpIndexes[i]];
	}

	// Get duration of bouts
	int duration_posneg[filteredAmpsSize];
	for (int i = 0; i < filteredAmpsSize; i++)
		duration_posneg[i] = filteredAmp_posneg[1][i] - filteredAmp_posneg[0][i];

	// Filter durations according to a threshold
	int bouts = 0;
	for (int i = 0; i < filteredAmpsSize; i++)
		if (duration_posneg[i] > BOUT_DURATION_THRESHOLD)
			bouts++;

	resetSamples();
	return bouts;
}

void Bout::convolute(const double signal[], const double kernel[], double result[])
{
	size_t signalLen = ELEMENT_COUNT(signal);
	size_t kernelLen = ELEMENT_COUNT(kernel);

	for (size_t n = 0; n < signalLen; n++)
	{
		result[n] = 0;
		size_t k_min = (n >= kernelLen - 1) ? n - (kernelLen - 1) : 0;
		size_t k_max = (k_min + kernelLen - 1 < signalLen - 1) ? k_min + kernelLen - 1 : signalLen - 1;

		for (size_t k = k_min; k < k_max; k++)
			result[n] += signal[k] * kernel[n - k];
	}
}

double Bout::gaussianValue(int i)
{
	int k = (KERNEL_LEN - 1) / 2;
	return 1 / (2 * M_PI * SMOOTH_STD) * exp(-1 * (i - k - 1) * (i - k - 1) / (2 * SMOOTH_STD));
}


void Bout::ewma(double signal[], double result[])
{
	size_t signalLen = sizeof(signal) / sizeof(signal[0]);

	double y_t_old = 0;
	double x_t = 0, x_t_old = 0;
	double alpha = 1 - exp(log(1 / (2 * HALF_LIFE * DELTA_TIME)));

	for (int i = 0; i < signalLen; i++)
	{
		x_t = signal[i];
		result[i] = (1 - alpha) * y_t_old + alpha * (x_t - x_t_old);

		x_t_old = signal[i];
		y_t_old = result[i];
	}
}

void Bout::resetSamples()
{
	for (int i = 0; i < MAX_NUM_SAMPLES; i++)
		signal[i] = 0;
	sampleIndex = 0;
}

void Bout::addSample(double sample)
{
	if (sampleIndex < MAX_NUM_SAMPLES)
	{
		signal[sampleIndex] = sample;
		sampleIndex++;
	}
	else
	{
		printf("Signal array is full\n");
	}
}

