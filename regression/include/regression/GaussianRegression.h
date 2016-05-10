/*
 * GaussianRegression.h
 *
 *  Created on: May 9, 2016
 *      Author: mikel
 */

#ifndef REGRESSION_SRC_GAUSSIANREGRESSION_H_
#define REGRESSION_SRC_GAUSSIANREGRESSION_H_

#include "regression/KernelFunction.h"
#include "regression/Position.h"

class GaussianRegression {
private:
	KernelFunction kernel;
	Position* X;
	int* y;
public:
	GaussianRegression(KernelFunction kernelFunction);
};

#endif /* REGRESSION_SRC_GAUSSIANREGRESSION_H_ */
