#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#define SIGNAL_LEN 10000
#define SMOOTH_STD 22 
#define KERNEL_LEN (int)10 * SMOOTH_STD
#define HALF_LIFE 29
#define DELTA_TIME 1
#define BOUT_AMP_THRESHOLD 0
#define BOUT_DURATION_THRESHOLD 0
#define CSV_LINE_LENGTH 130000


const char* getfield(char* line, int num);
void readSignal(FILE * file, double * signal);
int getBoutCount(double * signal);
void convolute(double * sig, const double kernel[], int kernelLen);
void convoluteInt(int * sig, const int kernel[], int kernelLen);
void populateGaussianFilter(double * kernel);
void ewma(double * sig);
void printArray(FILE * logFile, char const * name, double * array, int len);
void printArrayInt(FILE * logFile, char const * name, int * array, int len);
//double computeAmpThreshold(double * amps, int len);


void printArray(FILE * logFile, char const * name, double * array, int len)
{
	fprintf(logFile, "%s: ", name);
	int i;
	for (i = 0; i < len; i++)
	{
		fprintf(logFile, "%lf, ", array[i]);
	}
	fprintf(logFile, "\n");
}

void printArrayInt(FILE * logFile, char const * name, int * array, int len)
{
	fprintf(logFile, "%s: ", name);
	int i;
	for (i = 0; i < len; i++)
	{
		fprintf(logFile, "%d, ", array[i]);
	}
	fprintf(logFile, "\n");
}

const char* getfield(char* line, int num)
{
    	const char* tok;
    	for (tok = strtok(line, ",");
        	    tok && *tok;
            	tok = strtok(NULL, ",\n"))
   	{
        	if (!--num)
            		return tok;
    	}
    	return NULL;
}


void readSignal(FILE * file, double * signal)
{
    	char line[CSV_LINE_LENGTH];
	fgets(line, CSV_LINE_LENGTH, file);


	int i;
	for (i = 0; i < SIGNAL_LEN; i++)
	{
        	char * tmp = strdup(line);
		signal[i] = (double)atof(getfield(tmp, i+2));
        	free(tmp);
	}



}

int getBoutCount(double * signal)
{
	FILE * logFile = fopen("logz", "w");
	printArray(logFile, "Init signal", signal, SIGNAL_LEN);
	// Smooth
	double * gauss_kernel = (double*)malloc(KERNEL_LEN * sizeof(double));
	populateGaussianFilter(gauss_kernel);
	printArray(logFile, "Gauss kernel", gauss_kernel, KERNEL_LEN);

	convolute(signal, gauss_kernel, KERNEL_LEN);
	printArray(logFile, "After smoothing", signal, SIGNAL_LEN);

	free(gauss_kernel);

	// Diff
	double diff_kernel_double[2] = { -1, 1 };
	convolute(signal, diff_kernel_double, 2);
	printArray(logFile, "After diff", signal, SIGNAL_LEN);

	// EWMA
	ewma(signal);
	printArray(logFile, "After ewma", signal, SIGNAL_LEN);

	double * ewma_signal = (double*)malloc(SIGNAL_LEN * sizeof(double));
	memcpy(ewma_signal, signal, SIGNAL_LEN * sizeof(double));

	// Derivative
	convolute(signal, diff_kernel_double, 2);
	printArray(logFile, "Diff after ewma", signal, SIGNAL_LEN);

	// Positive part of the derivative
	int * sign_change = (int*)malloc(SIGNAL_LEN * sizeof(int));
	int i;
	for (i = 0; i < SIGNAL_LEN; i++)
		if (signal[i] >= 0)
			sign_change[i] = 1;
		else
			sign_change[i] = 0;

	printArrayInt(logFile, "init sign change", sign_change, SIGNAL_LEN);

	int diff_kernel_int[2] = { -1, 1};
	convoluteInt(sign_change, diff_kernel_int, 2);
	printArrayInt(logFile, "Diff int", sign_change, SIGNAL_LEN);
	// Get positive and negative indexes
	int * pos_changes = (int*)malloc(SIGNAL_LEN * sizeof(int));
	int * neg_changes = (int*)malloc(SIGNAL_LEN * sizeof(int));
	int pos_j = 0, neg_j = 0;

	for (i = 0; i < SIGNAL_LEN; i++)
	{
		if (sign_change[i] > 0)
			pos_changes[pos_j++] = i;
		if (sign_change[i] < 0)
			neg_changes[neg_j++] = i;
	}

	free(sign_change);



	int poslen = pos_j;
	int neglen = neg_j;

	printArrayInt(logFile, "Pos changes", pos_changes, poslen);
	printArrayInt(logFile, "Neg changes", neg_changes, neglen);


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

	for (i = 0; i < poslen; i++)
	{
		posneg[0][i] = pos_changes[i];
		posneg[1][i] = neg_changes[i];
	}
	free(pos_changes);
	free(neg_changes);

	// Get amplitudes
	double amps[poslen];

	for (i = 0; i < poslen; i++)
		amps[i] = ewma_signal[posneg[1][i]] - ewma_signal[posneg[0][i]];

	printArray(logFile, "Amps", amps, poslen);

	free(ewma_signal);

	// Filter amplitudes according to a threshold
	// double amp_threshold = computeAmpThreshold(amps, poslen);
	int superThreshAmpIndexes[poslen];
	int thresh_i = 0;
	/*
	for (i = 0; i < poslen; i++)
		if (amps[i] > amp_threshold)
			superThreshAmpIndexes[thresh_i++] = i;
	*/
	
	for (i = 0; i < poslen; i++)
		if (amps[i] > BOUT_AMP_THRESHOLD)
			superThreshAmpIndexes[thresh_i++] = i;
	
	int filteredAmpsSize = thresh_i;

	// Get indices of those that passed the threshold
	int filteredAmp_posneg[2][filteredAmpsSize];

	for (i = 0; i < filteredAmpsSize; i++)
	{
		filteredAmp_posneg[0][i] = posneg[0][superThreshAmpIndexes[i]];
		filteredAmp_posneg[1][i] = posneg[1][superThreshAmpIndexes[i]];
	}


	printArrayInt(logFile, "filteredAmp_posNeg0", filteredAmp_posneg[0], filteredAmpsSize);
	printArrayInt(logFile, "filteredAmp_posNeg1", filteredAmp_posneg[1], filteredAmpsSize);

	// Get duration of bouts
	int duration_posneg[filteredAmpsSize];

	for (i = 0; i < filteredAmpsSize; i++)
		duration_posneg[i] = filteredAmp_posneg[1][i] - filteredAmp_posneg[0][i];

	// Filter durations according to a threshold
	int bouts = 0;

	for (i = 0; i < filteredAmpsSize; i++)
		if (duration_posneg[i] > BOUT_DURATION_THRESHOLD)
			bouts++;
	

	fprintf(logFile, "\n\n\n");
	fclose(logFile);
	return bouts;
}


void convolute(double * sig, const double kernel[], int kernelLen)
{
	int halfKernelLen = kernelLen / 2;
	double * result = (double *)malloc(SIGNAL_LEN * sizeof(double));
	
	int n;
	for (n = 0; n < SIGNAL_LEN; n++)
	{
		result[n] = 0;

		int k_min = (n < halfKernelLen ? 0 : n - halfKernelLen );
		int k_max = (n < SIGNAL_LEN - halfKernelLen ? n + halfKernelLen : SIGNAL_LEN);
		int k;
		for (k = k_min; k < k_max; k++)
			result[n] += sig[k] * kernel[halfKernelLen - n + k];
	}
	memcpy(sig, result, SIGNAL_LEN * sizeof(double));
	free(result);
}


void convoluteInt(int * sig, const int kernel[], int kernelLen)
{
	int halfKernelLen = kernelLen / 2;
	int * result = (int *)malloc(SIGNAL_LEN * sizeof(int));
	int n;
	for (n = 0; n < SIGNAL_LEN; n++)
	{
		result[n] = 0;

		int k_min = (n < halfKernelLen ? 0 : n - halfKernelLen );
		int k_max = (n < SIGNAL_LEN - halfKernelLen ? n + halfKernelLen : SIGNAL_LEN);
		int k;
		for (k = k_min; k < k_max; k++)
			result[n] += sig[k] * kernel[halfKernelLen - n + k];
	}
	memcpy(sig, result, SIGNAL_LEN * sizeof(int));
	free(result);
}



void populateGaussianFilter(double * kernel)
{
	double sum = 0;
	double r = 0;
	int i;
	for (i = -KERNEL_LEN / 2; i < KERNEL_LEN / 2; i++)
	{
		r = sqrt(i * i);
//		kernel[i + KERNEL_LEN / 2] = exp(-(r * r)/(2 * SMOOTH_STD * SMOOTH_STD)) / (sqrt(2 * M_PI) * SMOOTH_STD);
		kernel[i + KERNEL_LEN / 2] = exp(-0.5 * (r * r) / (SMOOTH_STD * SMOOTH_STD));
		sum += kernel[i + KERNEL_LEN / 2];
	}

	for (i = 0; i < KERNEL_LEN; i++)
	{
		kernel[i] /= sum;
	}
}


void ewma(double * sig)
{
	double * result = (double*)malloc(SIGNAL_LEN * sizeof(double));

	double y_t_old = 0;
	double x_t = 0, x_t_old = 0;
	double alpha = 1 - exp(log(0.5) / (HALF_LIFE * DELTA_TIME));
	int i;
	for (i = 0; i < SIGNAL_LEN; i++)
	{
		x_t = sig[i];
		//result[i] = (1 - alpha) * y_t_old + alpha * (x_t - x_t_old);
		result[i] = (1 - alpha) * y_t_old + alpha * (x_t);

		x_t_old = sig[i];
		y_t_old = result[i];
	}
	memcpy(sig, result, SIGNAL_LEN * sizeof(double));
	free(result);

}

/*
double computeAmpThreshold(double * amps, int len)
{
	double mean = 0.0, std = 0.0;
	int i;
	for (i = 0; i < len; i++)
		mean += amps[i];
	mean /= len;

	for (i = 0; i < len; i++)
		std += (amps[i] - mean) * (amps[i] - mean);
	std /= len;
	return mean * 3 * sqrt(std);
}
*/
int main()
{
	double * signal = (double *)malloc(SIGNAL_LEN * sizeof(double));
	FILE * file = fopen("boutLogs.csv", "r");
	int i;
	for (i = 0; i < 6; i++)
	{
		readSignal(file, signal);
		printf("Bouts: %d\n", getBoutCount(signal));
	}
	fclose(file);

	free(signal);
}
