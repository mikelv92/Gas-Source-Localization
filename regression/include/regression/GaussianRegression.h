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
#include "regression/Utilities.h"

#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>
#include <stdlib.h>

#include <Eigen/Dense>
#include <list>
#include <map>

#define SIGMA_N 2;
#define ENV_X 10
#define ENV_Y 5
#define STEP_SIZE 0.5
#define A_K 100

using namespace Eigen;
using namespace std;
using namespace boost;

class GaussianRegression {
private:
	double alpha; // Tradeoff between variance and mean
	KernelFunction * kernel;
	list<Position> X;

	MatrixXd K;
	VectorXd y;

	bool isExplored(Position x);
	Position computeCentroid(map<Position, double> dataMap);
	double computeOrientationToFollow(Position meanPos, Position varPos);

public:
	GaussianRegression();
	double mean(Position x_star);
	double variance(Position x_star);
	void addMeasurement(Position x_prime, int boutCount);
	Position nextBestPosition();
	void setKernel(KernelFunction * kernelFunction);
	void writeMeanMap(FILE * logFile);
};

#endif /* REGRESSION_SRC_GAUSSIANREGRESSION_H_ */
