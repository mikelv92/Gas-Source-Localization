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


	//Read how many 0-1 switches are.
	double finalSignal[MAX_NUM_SAMPLES];
	convolute(ewmaSignal, diff_kernel, finalSignal);

	for (int i = 0; i < MAX_NUM_SAMPLES; i++)
		if (finalSignal[i] >= 0)
			finalSignal[i] = 1;
		else
			finalSignal[i] = -1;

	int bouts = 1;

	resetSamples();
	return bouts;
}

void Bout::convolute(const double signal[], const double kernel[], double result[])
{
	size_t signalLen = sizeof(signal) / sizeof(signal[0]);
	size_t kernelLen = sizeof(kernel) / sizeof(kernel[0]);

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

