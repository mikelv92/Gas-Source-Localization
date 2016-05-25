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
#include <map>

#define SIGMA_N 5;
#define ENV_X 61
#define ENV_Y 21
#define STEP_SIZE 1
#define A_K 100

using namespace Eigen;
using namespace std;

class GaussianRegression {
private:
	KernelFunction * kernel;
	list<Position> X;

	MatrixXd K;
	VectorXd y;

	bool isExplored(Position x);
	void addElementToVector(VectorXd * vector, double element);
	void addElementToVector(RowVectorXd * vector, double element);

public:
	GaussianRegression();
	double mean(Position x_star);
	double variance(Position x_star);
	void addMeasurement(Position x_prime, int boutCount);
	Position nextBestPosition();
	void setKernel(KernelFunction * kernelFunction);
};

#endif /* REGRESSION_SRC_GAUSSIANREGRESSION_H_ */
