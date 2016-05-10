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

	float semiMajorAxis;
	float semiMinorAxis;

	float sigma0Upwind[2][2];
	float sigma0Downwind[2][2];

	float rotMatrix[2][2];

	float sigmaUpwind[2][2];
	float sigmaDownwind[2][2];

	float** invertMatrix(float matrix[2][2]);
	bool isUpwind(Position diff);

public:
	KernelFunction() {}
	KernelFunction(Wind w);

	float getK(Position x, Position x_prime);

	virtual ~KernelFunction();
};

#endif /* REGRESSION_SRC_KERNELFUNCTION_H_ */
