/*
 * Bout.cpp
 *
 *  Created on: May 9, 2016
 *      Author: mikel
 */

#include "regression/Bout.h"

Bout::Bout(FILE * f) {
	sampleIndex = 0;
	signal = (double*)malloc(SIGNAL_LEN * sizeof(double));
	resetSamples();

	logFile = f;
}
void Bout::printArray(char const * name, double * array, int len)
{
	fprintf(logFile, "%s: ", name);
	for (int i = 0; i < len; i++)
	{
		fprintf(logFile, "%lf, ", array[i]);
	}
	fprintf(logFile, "\n");
}

void Bout::printArray(char const * name, int * array, int len)
{
	fprintf(logFile, "%s: ", name);
	for (int i = 0; i < len; i++)
	{
		fprintf(logFile, "%d, ", array[i]);
	}
	fprintf(logFile, "\n");
}

int Bout::getBoutCount()
{
	//printArray("Init signal", signal, SIGNAL_LEN);
	// Smooth
	double * gauss_kernel = (double*)malloc(KERNEL_LEN * sizeof(double));
	populateGaussianFilter(gauss_kernel);
	//printArray("Gauss kernel", gauss_kernel, KERNEL_LEN);

	convolute(signal, gauss_kernel, KERNEL_LEN);

	printArray("After smoothing", signal, SIGNAL_LEN);

	free(gauss_kernel);

	// Diff
	double diff_kernel_double[2] = { 1, -1 };
	convolute(signal, diff_kernel_double, 2);
	printArray("After diff", signal, SIGNAL_LEN);

	// EWMA
	ewma(signal);

	printArray("After ewma", signal, SIGNAL_LEN);

	double * ewma_signal = (double*)malloc(SIGNAL_LEN * sizeof(double));
	memcpy(ewma_signal, signal, SIGNAL_LEN * sizeof(double));

	// Derivative
	convolute(signal, diff_kernel_double, 2);
	printArray("Diff after ewma", signal, SIGNAL_LEN);


	// Positive part of the derivative
	int * sign_change = (int*)malloc(SIGNAL_LEN * sizeof(int));
	for (int i = 0; i < SIGNAL_LEN; i++)
		if (signal[i] >= 0)
			sign_change[i] = 1;
		else
			sign_change[i] = 0;

	printArray("init sign change", sign_change, SIGNAL_LEN);


	int diff_kernel_int[2] = { 1, -1};
	convolute(sign_change, diff_kernel_int, 2);
	printArray("Diff int", sign_change, SIGNAL_LEN);

	// Get positive and negative indexes
	int * pos_changes = (int*)malloc(SIGNAL_LEN * sizeof(int));
	int * neg_changes = (int*)malloc(SIGNAL_LEN * sizeof(int));
	int pos_j = 0, neg_j = 0;
	for (int i = 0; i < SIGNAL_LEN; i++)
	{
		if (sign_change[i] > 0)
			pos_changes[pos_j++] = i;
		if (sign_change[i] < 0)
			neg_changes[neg_j++] = i;
	}

	free(sign_change);

	int poslen = pos_j;
	int neglen = neg_j;

	printArray("Pos changes", pos_changes, poslen);
	printArray("Neg changes", neg_changes, neglen);

	int * neg_changes_2;
	if (pos_changes[0] > neg_changes[0])
	{
		neg_changes_2 = neg_changes + sizeof(int); //discard first negative change
		neglen--;
	}
	else
		neg_changes_2 = neg_changes;

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

	printArray("Amps", amps, poslen);

	free(ewma_signal);

	// Filter amplitudes according to a threshold
	double amp_threshold = computeAmpThreshold(amps, poslen);
	printf("Amp thesh: %lf\n", amp_threshold);
	int superThreshAmpIndexes[poslen];
	int thresh_i = 0;
	for (int i = 0; i < poslen; i++)
		if (amps[i] > amp_threshold)
			superThreshAmpIndexes[thresh_i++] = i;
	int filteredAmpsSize = thresh_i;

	// Get indices of those that passed the threshold
	int filteredAmp_posneg[2][filteredAmpsSize];
	for (int i = 0; i < filteredAmpsSize; i++)
	{
		filteredAmp_posneg[0][i] = posneg[0][superThreshAmpIndexes[i]];
		filteredAmp_posneg[1][i] = posneg[1][superThreshAmpIndexes[i]];
	}
	printArray("filteredAmp_posNeg0", filteredAmp_posneg[0], filteredAmpsSize);
	printArray("filteredAmp_posNeg1", filteredAmp_posneg[1], filteredAmpsSize);

	// Get duration of bouts
	int duration_posneg[filteredAmpsSize];
	for (int i = 0; i < filteredAmpsSize; i++)
		duration_posneg[i] = filteredAmp_posneg[1][i] - filteredAmp_posneg[0][i];

	printArray("duration_posneg", duration_posneg, filteredAmpsSize);

	// Filter durations according to a threshold
	int bouts = 0;
	for (int i = 0; i < filteredAmpsSize; i++)
		if (duration_posneg[i] > BOUT_DURATION_THRESHOLD)
			bouts++;

	resetSamples();


	return bouts;
}


void Bout::convolute(double * sig, const double kernel[], int kernelLen)
{
	double * result = (double*)malloc(SIGNAL_LEN * sizeof(double));

	int i, j, k;
	for (i = kernelLen - 1; i < SIGNAL_LEN; i++)
	{
		result[i] = 0;
		for (j = i, k = 0; k < kernelLen; j--, k++)
			result[i] += sig[j] * kernel[k];
	}

	for (i = 0; i < kernelLen - 1; i++)
	{
		result[i] = 0;
		for (j = i, k = 0; j >= 0; j--, k++)
			result[i] += sig[j] * kernel[k];
	}

	memcpy(sig, result, SIGNAL_LEN * sizeof(double));
	free(result);
}

void Bout::convolute(int * sig, const int kernel[], int kernelLen)
{
	int * result = (int*)malloc(SIGNAL_LEN * sizeof(int));

	int i, j, k;
	for (i = kernelLen - 1; i < SIGNAL_LEN; i++)
	{
		result[i] = 0;
		for (j = i, k = 0; k < kernelLen; j--, k++)
			result[i] += sig[j] * kernel[k];
	}

	for (i = 0; i < kernelLen - 1; i++)
	{
		result[i] = 0;
		for (j = i, k = 0; j >= 0; j--, k++)
			result[i] += sig[j] * kernel[k];
	}

	memcpy(sig, result, SIGNAL_LEN * sizeof(int));
	free(result);

}


void Bout::populateGaussianFilter(double * kernel)
{
	double sum = 0;
	double r = 0;
	for (int i = -KERNEL_LEN / 2; i < KERNEL_LEN / 2; i++)
	{
		r = sqrt(i * i);
		kernel[i + KERNEL_LEN / 2] = exp(-(r * r)/(2 * SMOOTH_STD * SMOOTH_STD)) / (sqrt(2 * M_PI) * SMOOTH_STD);
		sum += kernel[i + KERNEL_LEN / 2];
	}

	for (int i = 0; i < KERNEL_LEN; i++)
	{
		kernel[i] /= sum;
	}
}


void Bout::ewma(double * sig)
{
	double * result = (double*)malloc(SIGNAL_LEN * sizeof(double));

	double y_t_old = 0;
	double x_t = 0, x_t_old = 0;
	double alpha = 1 - exp(log(1 / (2 * HALF_LIFE * DELTA_TIME)));

	for (int i = 0; i < SIGNAL_LEN; i++)
	{
		x_t = sig[i];
		result[i] = (1 - alpha) * y_t_old + alpha * (x_t - x_t_old);

		x_t_old = sig[i];
		y_t_old = result[i];
	}
	memcpy(sig, result, SIGNAL_LEN * sizeof(double));
	free(result);

}

double Bout::computeAmpThreshold(double * amps, int len)
{
	double mean = 0.0, std = 0.0;
	for (int i = 0; i < len; i++)
		mean += amps[i];
	mean /= len;

	for (int i = 0; i < len; i++)
		std += (amps[i] - mean) * (amps[i] - mean);
	std /= len;
	return mean * 3 * sqrt(std);
}

void Bout::resetSamples()
{
	for (int i = 0; i < SIGNAL_LEN; i++)
		signal[i] = 0;
	sampleIndex = 0;
}

void Bout::addSample(double sample)
{
	if (sampleIndex < SIGNAL_LEN)
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
