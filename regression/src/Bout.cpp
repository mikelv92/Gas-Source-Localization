/*
 * Bout.cpp
 *
 *  Created on: May 9, 2016
 *      Author: mikel
 */

#include "regression/Bout.h"

Bout::Bout() {
	sampleIndex = 0;
	signal = (double*)malloc(MAX_NUM_SAMPLES * sizeof(double));
	resetSamples();
}
void printArray(char * name, double * array)
{
	printf("%s: ", name);
	for (int i = 0; i < Bout::MAX_NUM_SAMPLES; i++)
	{
		printf("%lf, ", array[i]);
	}
	printf("\n");
}

void printArray(char * name, int * array)
{
	printf("%s: ", name);
	for (int i = 0; i < Bout::MAX_NUM_SAMPLES; i++)
	{
		printf("%d, ", array[i]);
	}
	printf("\n");
}

int Bout::getBoutCount()
{
	// Smooth
	double gauss_kernel[KERNEL_LEN];
	for (int i = 0; i < KERNEL_LEN; i++)
	{
		gauss_kernel[i] = gaussianValue(i);
	}

	printArray("signal before smooth", signal);
	convolute(signal, gauss_kernel);
	printArray("signal after smooth", signal);

	// Diff
	double diff_kernel_double[2] = { -1, 1};
	convolute(signal, diff_kernel_double);
	//signal += sizeof(double); //Remove first element as it is the same as the second element when convoluting with differential kernel

	// EWMA
	ewma(signal);

	double * ewma_signal = (double*)malloc(ELEMENT_COUNT(signal) * sizeof(double));
	memcpy(ewma_signal, signal, sizeof(signal));

	// Derivative
	convolute(signal, diff_kernel_double);
	//signal += sizeof(double);

	// Positive part of the derivative
	int * sign_change = (int*)malloc(ELEMENT_COUNT(signal) * sizeof(int));
	for (int i = 0; i < ELEMENT_COUNT(signal); i++)
		if (signal[i] >= 0)
			sign_change[i] = 1;
		else
			sign_change[i] = 0;

	int diff_kernel_int[2] = { -1, 1};
	convolute(sign_change, diff_kernel_int);
	//sign_change += sizeof(int);

	// Get positive and negative indexes
	int * pos_changes = (int*)malloc(ELEMENT_COUNT(sign_change) * sizeof(int));
	int * neg_changes = (int*)malloc(ELEMENT_COUNT(sign_change) * sizeof(int));
	int pos_j = 0, neg_j = 0;
	for (int i = 0; i < ELEMENT_COUNT(signal); i++)
	{
		if (sign_change[i] > 0)
			pos_changes[pos_j++] = i;
		if (sign_change[i] < 0)
			neg_changes[neg_j++] = i;
	}

	free(sign_change);
//	if (pos_changes[0] > neg_changes[0])
//		neg_changes += sizeof(int); //discard first negative change

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

	free(pos_changes);
	free(neg_changes);


	// Get amplitudes
	double amps[poslen];
	for (int i = 0; i < poslen; i++)
		amps[i] = ewma_signal[posneg[1][i]] - ewma_signal[posneg[0][i]];

	free(ewma_signal);

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


void Bout::convolute(double * sig, const double kernel[])
{
	size_t signalLen = ELEMENT_COUNT(sig);
	size_t kernelLen = ELEMENT_COUNT(kernel);
	double * result = (double*)malloc(signalLen * sizeof(double));

	for (size_t n = 0; n < signalLen; n++)
	{
		result[n] = 0;
		size_t k_min = (n >= kernelLen - 1) ? n - (kernelLen - 1) : 0;
		size_t k_max = (k_min + kernelLen - 1 < signalLen - 1) ? k_min + kernelLen - 1 : signalLen - 1;

		for (size_t k = k_min; k < k_max; k++)
			result[n] += sig[k] * kernel[n - k];
	}
	memcpy(sig, result, sizeof(result));
	free(result);
}

void Bout::convolute(int * sig, const int kernel[])
{
	size_t signalLen = ELEMENT_COUNT(sig);
	size_t kernelLen = ELEMENT_COUNT(kernel);
	int * result = (int*)malloc(signalLen * sizeof(int));

	for (size_t n = 0; n < signalLen; n++)
	{
		result[n] = 0;
		size_t k_min = (n >= kernelLen - 1) ? n - (kernelLen - 1) : 0;
		size_t k_max = (k_min + kernelLen - 1 < signalLen - 1) ? k_min + kernelLen - 1 : signalLen - 1;

		for (size_t k = k_min; k < k_max; k++)
			result[n] += sig[k] * kernel[n - k];
	}
	memcpy(sig, result, sizeof(result));
	free(result);

}


double Bout::gaussianValue(int i)
{
	int k = (KERNEL_LEN - 1) / 2;
	return 1 / (2 * M_PI * SMOOTH_STD) * exp(-1 * (i - k - 1) * (i - k - 1) / (2 * SMOOTH_STD));
}


void Bout::ewma(double * sig)
{
	size_t signalLen = sizeof(sig) / sizeof(sig[0]);
	double * result = (double*)malloc(signalLen * sizeof(double));

	double y_t_old = 0;
	double x_t = 0, x_t_old = 0;
	double alpha = 1 - exp(log(1 / (2 * HALF_LIFE * DELTA_TIME)));

	for (int i = 0; i < signalLen; i++)
	{
		x_t = sig[i];
		result[i] = (1 - alpha) * y_t_old + alpha * (x_t - x_t_old);

		x_t_old = sig[i];
		y_t_old = result[i];
	}
	memcpy(sig, result, sizeof(result));
	free(result);

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



Bout::~Bout()
{
	if (signal)
		free(signal);
}
