/*
 * Utilities.h
 *
 *  Created on: Jun 16, 2016
 *      Author: mikel
 */

#ifndef GAS_SOURCE_LOCALIZATION_REGRESSION_INCLUDE_REGRESSION_UTILITIES_H_
#define GAS_SOURCE_LOCALIZATION_REGRESSION_INCLUDE_REGRESSION_UTILITIES_H_

#include "regression/Position.h"
#include <Eigen/Dense>
#include <map>
using namespace Eigen;
using namespace std;

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

	static void addElementToVector(VectorXd * vector, double element)
	{
		vector->conservativeResize(vector->rows() + 1);
		RowVectorXd vec(1);
		vec << element;
		vector->row(vector->rows() - 1) = vec;
	}

	static void addElementToVector(RowVectorXd * vector, double element)
	{
		vector->conservativeResize(vector->cols() + 1);
		VectorXd vec(1);
		vec << element;
		vector->col(vector->cols() - 1) = vec;
	}

	static Position computeCentroid(map<Position, double> dataMap)
	{
		double numX = 0, numY = 0, den = 0;
		for (map<Position, double>::iterator it = dataMap.begin(); it != dataMap.end(); it++)
		{
			numX 	+= it->first.getX() * it->second;
			numY 	+= it->first.getY() * it->second;
			den 	+= it->second;
		}
		return Position(floor(numX / den), floor(numY / den));
	}

	static double integral(double(*f)(double x), double a, double b, int n)
	{
		//a is low bound, b is high bound, x is the variable, n is the steps
		double step = (b - a) / n;
		double area = 0.0;
		for (int i = 0; i < n; i++)
		{
			area += f(a + (i + 0.5) * step) * step;
		}
		return area;
	}




};

#endif /* GAS_SOURCE_LOCALIZATION_REGRESSION_INCLUDE_REGRESSION_UTILITIES_H_ */
