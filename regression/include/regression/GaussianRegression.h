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

#include <cmath>

#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>
#include <stdlib.h>

#include <Eigen/Dense>
#include <list>
#include <map>

#define SIGMA_N 2
#define ENV_X 10
#define ENV_Y 5
#define STEP_SIZE 1
#define INIT_ALPHA 100
#define RHO 2.0
#define MEAN_GAUSS_VARIANCE 2.0
#define VAR_GAUSS_VARIANCE 2.0
#define ALPHA_THRESHOLD 0.4
#define EXPLORE_X 5
#define EXPLORE_Y 5

using namespace Eigen;
using namespace std;
using namespace boost;

class GaussianRegression {
private:
	double alpha; // Tradeoff between variance and mean
	double meanAngle;
	double varAngle;

	KernelFunction * kernel;
	GMap * gmap;
	list<Position> X;

	MatrixXd K;
	VectorXd y;

	Position currentPosition;

	map<Position, double> globalMeanMap;
	map<Position, double> globalVarianceMap;

	bool isExplored(Position x);
	Position updateCurrentPosition(Position meanPos, Position varPos);
	Position updatePosToNearestFreeCell(Position position);
	Position getMaxMeanPos();
	Position getMaxVariancePos();
	Position finalPosition();
	double meanDirGaussF(double theta);
	double varDirGaussF(double theta);

public:
	GaussianRegression();
	double mean(Position x_star);
	double variance(Position x_star);
	void addMeasurement(Position x_prime, double value);
	Position nextBestPosition();
	void setKernel(KernelFunction * kernelFunction);
	void setGMap(GMap * gmap);

	void setCurrentPosition(Position position)
	{
		this->currentPosition = position;
	}

	~GaussianRegression();
};

#endif /* REGRESSION_SRC_GAUSSIANREGRESSION_H_ */
