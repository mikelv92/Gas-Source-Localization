/*
 * Utilities.h
 *
 *  Created on: Jun 16, 2016
 *      Author: mikel
 */

#ifndef GAS_SOURCE_LOCALIZATION_REGRESSION_INCLUDE_REGRESSION_UTILITIES_H_
#define GAS_SOURCE_LOCALIZATION_REGRESSION_INCLUDE_REGRESSION_UTILITIES_H_

class Utilities {
public:
	Utilities();

	static void printArray(FILE * logFile, char const * name, double * array, int len)
	{
		fprintf(logFile, "%s: ", name);
		for (int i = 0; i < len; i++)
		{
			fprintf(logFile, "%lf, ", array[i]);
		}
		fprintf(logFile, "\n");
	}

	static void printArray(FILE * logFile, char const * name, int * array, int len)
	{
		fprintf(logFile, "%s: ", name);
		for (int i = 0; i < len; i++)
		{
			fprintf(logFile, "%d, ", array[i]);
		}
		fprintf(logFile, "\n");
	}

};

#endif /* GAS_SOURCE_LOCALIZATION_REGRESSION_INCLUDE_REGRESSION_UTILITIES_H_ */
