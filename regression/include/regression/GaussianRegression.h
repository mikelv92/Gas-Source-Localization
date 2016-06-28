/*
 * GaussianRegression.h
 *
 *  Created on: May 9, 2016
 *      Author: mikel
 */

#ifndef REGRESSION_SRC_GAUSSIANREGRESSION_H_
#define REGRESSION_SRC_GAUSSIANREGRESSION_H_

#include "regression/KernelFunction.h"
#include "regression/GMap.h"
#include "regression/Position.h"
#include "regression/Utilities.h"

#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>
#include <stdlib.h>

#include <Eigen/Dense>
#include <list>
#include <map>

#define SIGMA_N 2
#define ENV_X 10
#define ENV_Y 5
#define STEP_SIZE 0.5
#define A_K 100
#define RHO 5.0
#define MEAN_GAUSS_VARIANCE 2.0
#define VAR_GAUSS_VARIANCE 2.0
#define ALPHA_THRESHOLD 0.1

using namespace Eigen;
using namespace std;
using namespace boost;

class GaussianRegression {
private:
	double alpha; // Tradeoff between variance and mean
	KernelFunction * kernel;
	GMap * gmap;
	list<Position> X;

	MatrixXd K;
	VectorXd y;

	Position currentPosition;

	bool isExplored(Position x);
	Position computeCentroid(map<Position, double> dataMap);
	Position updateCurrentPosition(Position meanPos, Position varPos);

public:
	GaussianRegression();
	double mean(Position x_star);
	double variance(Position x_star);
	void addMeasurement(Position x_prime, int boutCount);
	Position nextBestPosition();
	void setKernel(KernelFunction * kernelFunction);
	void setGMap(GMap * gmap);
	void writeMeanMap(FILE * logFile);
};

#endif /* REGRESSION_SRC_GAUSSIANREGRESSION_H_ */
