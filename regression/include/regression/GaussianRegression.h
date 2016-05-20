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
#include <Eigen/Dense>
#include <list>

#define SIGMA_N 5;

using namespace Eigen;
using namespace std;

class GaussianRegression {
private:
	KernelFunction kernel;
	list<Position> X;

	MatrixXd K;
	VectorXd y;

public:
	GaussianRegression(KernelFunction kernelFunction);
	double mean(Position x_star);
	double variance(Position x_star);
	void addMeasurement(Position x_prime, int boutCount);
	~GaussianRegression();
};

#endif /* REGRESSION_SRC_GAUSSIANREGRESSION_H_ */
