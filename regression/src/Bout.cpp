/*
 * Bout.cpp
 *
 *  Created on: May 9, 2016
 *      Author: mikel
 */

#include "regression/Bout.h"

Bout::Bout() {
	for (int i = PID; i != NUM_SIGNALS; i++)
	{
		SignalIndex index = static_cast<SignalIndex>(i);

		sampleIndexes[i] = 0;
		double * signal = (double*)malloc(SIGNAL_LEN * sizeof(double));
		signalsMap[index] = signal;
		resetSamples(index);
	}

	logFile = fopen("boutLogs", "w");
	if (logFile == NULL)
	{
		printf("Error opening bout log file\n");
		exit(1);
	}
}

void Bout::computeBouts()
{
	for (int i = S1; i != NUM_SIGNALS; i++)
	{
		SignalIndex signalIndex = static_cast<SignalIndex>(i);

		double * signal = signalsMap.find(signalIndex)->second;

		// Smooth
		double * gauss_kernel = (double*)malloc(KERNEL_LEN * sizeof(double));
		populateGaussianFilter(gauss_kernel);

		convolute(signal, gauss_kernel, KERNEL_LEN);

		free(gauss_kernel);

		// Diff
		double diff_kernel_double[2] = { -1, 1 };
		convolute(signal, diff_kernel_double, 2);

		// EWMA
		ewma(signal);

		double * ewma_signal = (double*)malloc(SIGNAL_LEN * sizeof(double));
		memcpy(ewma_signal, signal, SIGNAL_LEN * sizeof(double));

		// Derivative
		convolute(signal, diff_kernel_double, 2);

		// Positive part of the derivative
		int * sign_change = (int*)malloc(SIGNAL_LEN * sizeof(int));
		for (int i = 0; i < SIGNAL_LEN; i++)
			if (signal[i] >= 0)
				sign_change[i] = 1;
			else
				sign_change[i] = 0;

		int diff_kernel_int[2] = { -1, 1};
		convolute(sign_change, diff_kernel_int, 2);

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

		free(ewma_signal);

		// Filter amplitudes according to a threshold
		//	double amp_threshold = computeAmpThreshold(amps, poslen);
		//	printf("Amp thesh: %lf\n", amp_threshold);
		int superThreshAmpIndexes[poslen];
		int thresh_i = 0;
		//	for (int i = 0; i < poslen; i++)
		//		if (amps[i] > amp_threshold)
		//			superThreshAmpIndexes[thresh_i++] = i;
		for (int i = 0; i < poslen; i++)
			if (amps[i] > BOUT_AMP_THRESHOLD)
				superThreshAmpIndexes[thresh_i++] = i;

		int filteredAmpsSize = thresh_i;

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

		boutsMap[signalIndex] = bouts;

		double ampsAvg = 0;
		for (int k = 0; k < poslen; k++)
			ampsAvg += amps[k];
		amplitudeMap[signalIndex] = ampsAvg / poslen;
	}
}

int Bout::getBoutCount(SignalIndex signalIndex)
{
	return boutsMap.find(signalIndex)->second;
}

double Bout::getAmplitude(SignalIndex signalIndex)
{
	return amplitudeMap.find(signalIndex)->second;
}

int Bout::getBoutCountAverage()
{
	int boutAverage = 0;
	for (map<SignalIndex, int>::iterator it = boutsMap.begin(); it != boutsMap.end(); it++)
		boutAverage += it->second;
	return boutAverage / boutsMap.size();
}

double Bout::getAmplitudeAverage()
{
	double ampAverage = 0;
	for (map<SignalIndex, double>::iterator it = amplitudeMap.begin(); it != amplitudeMap.end(); it++)
		ampAverage += it->second;
	return ampAverage / amplitudeMap.size();
}

void Bout::convolute(double * sig, const double kernel[], int kernelLen)
{
	int halfKernelLen = kernelLen / 2;
	double * result = (double *)malloc(SIGNAL_LEN * sizeof(double));

	for (int n = 0; n < SIGNAL_LEN; n++)
	{
		result[n] = 0;

		int k_min = (n < halfKernelLen ? 0 : n - halfKernelLen );
		int k_max = (n < SIGNAL_LEN - halfKernelLen ? n + halfKernelLen : SIGNAL_LEN);

		for (int k = k_min; k < k_max; k++)
			result[n] += sig[k] * kernel[halfKernelLen - n + k];
	}
	memcpy(sig, result, SIGNAL_LEN * sizeof(double));
	free(result);
}

void Bout::convolute(int * sig, const int kernel[], int kernelLen)
{
	int halfKernelLen = kernelLen / 2;
	int * result = (int *)malloc(SIGNAL_LEN * sizeof(int));

	for (int n = 0; n < SIGNAL_LEN; n++)
	{
		result[n] = 0;

		int k_min = (n < halfKernelLen ? 0 : n - halfKernelLen );
		int k_max = (n < SIGNAL_LEN - halfKernelLen ? n + halfKernelLen : SIGNAL_LEN);

		for (int k = k_min; k < k_max; k++)
			result[n] += sig[k] * kernel[halfKernelLen - n + k];
	}
	memcpy(sig, result, SIGNAL_LEN * sizeof(int));
	free(result);
}

/*
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
 */

void Bout::populateGaussianFilter(double * kernel)
{
	double sum = 0;
	double r = 0;
	for (int i = -KERNEL_LEN / 2; i < KERNEL_LEN / 2; i++)
	{
		r = sqrt(i * i);
		//		kernel[i + KERNEL_LEN / 2] = exp(-(r * r)/(2 * SMOOTH_STD * SMOOTH_STD)) / (sqrt(2 * M_PI) * SMOOTH_STD);
		kernel[i + KERNEL_LEN / 2] = exp(-0.5 * (r * r) / (SMOOTH_STD * SMOOTH_STD));
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
	double alpha = 1 - exp(log(0.5) / (HALF_LIFE * DELTA_TIME));

	for (int i = 0; i < SIGNAL_LEN; i++)
	{
		x_t = sig[i];
		result[i] = (1 - alpha) * y_t_old + alpha * (x_t);

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

void Bout::resetSamples(SignalIndex signalIndex)
{
	map<SignalIndex, double *>::iterator it = signalsMap.find(signalIndex);

	for (int i = 0; i < SIGNAL_LEN; i++)
		it->second[i] = 0;

	sampleIndexes[it->first] = 0;
}

void Bout::addSample(SignalIndex signalIndex, double sample)
{
	if (sampleIndexes[signalIndex] < SIGNAL_LEN)
	{
		map<SignalIndex, double *>::iterator it = signalsMap.find(signalIndex);

		it->second[sampleIndexes[signalIndex]] = sample;
		sampleIndexes[signalIndex]++;
	}
	else
	{
		printf("Signal array is full\n");
	}
}

bool Bout::isSignalArrayFull(SignalIndex signalIndex)
{
	return sampleIndexes[signalIndex] >= SIGNAL_LEN;
}

Bout::~Bout()
{
	for (map<SignalIndex, double *>::iterator it = signalsMap.begin(); it != signalsMap.end(); it++)
	{
		if (it->second)
			free(it->second);
	}

	if (logFile)
		fclose(logFile);
}
