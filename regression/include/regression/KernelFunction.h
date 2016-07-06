/*
 * KernelFunction.h
 *
 *  Created on: May 9, 2016
 *      Author: mikel
 */

#ifndef REGRESSION_SRC_KERNELFUNCTION_H_
#define REGRESSION_SRC_KERNELFUNCTION_H_
#define SPATIAL_SCALE 1.2
#define WIND_SCALE 0.2

#include <cmath>
#include <stdio.h>
#include <cstdlib>
#include "regression/Wind.h"
#include "regression/Position.h"

class KernelFunction {
private:
	Wind wind;

	double semiMajorAxis;
	double semiMinorAxis;

	double ** sigmaUpwind;
	double ** sigmaDownwind;

	void invertMatrix(double ** matrix);
	bool isUpwind(Position diff);

public:
	KernelFunction(Wind w);

	double getK(Position x, Position x_prime);
	~KernelFunction();

	const Wind& getWind() const {
		return wind;
	}

	void setWind(const Wind& wind) {
		this->wind = wind;
	}
};

#endif /* REGRESSION_SRC_KERNELFUNCTION_H_ */
