/*
 * KernelFunction.h
 *
 *  Created on: May 9, 2016
 *      Author: mikel
 */

#ifndef REGRESSION_SRC_KERNELFUNCTION_H_
#define REGRESSION_SRC_KERNELFUNCTION_H_
#define SPATIAL_SCALE 0.5
#define WIND_SCALE 0.5
#define REDUCE_FACTOR 10

#include <cmath>
#include "regression/Wind.h"
#include "regression/Position.h"

class KernelFunction {
private:
	Wind wind;

	double semiMajorAxis;
	double semiMinorAxis;

	double sigma0Upwind[2][2];
	double sigma0Downwind[2][2];

	double rotMatrix[2][2];

	double sigmaUpwind[2][2];
	double sigmaDownwind[2][2];

	double** invertMatrix(double matrix[2][2]);
	bool isUpwind(Position diff);

public:
	KernelFunction() {}
	KernelFunction(Wind w);

	double getK(Position x, Position x_prime);

};

#endif /* REGRESSION_SRC_KERNELFUNCTION_H_ */
