/*
 * KernelFunction.cpp
 *
 *  Created on: May 9, 2016
 *      Author: mikel
 */

#include "regression/KernelFunction.h"

KernelFunction::KernelFunction(Wind w) {
	wind = w;

	// Simulator wind
	/*
	semiMajorAxis = SPATIAL_SCALE + WIND_SCALE * wind.get2DSpeed();
	semiMinorAxis = SPATIAL_SCALE / (1 + (WIND_SCALE * wind.get2DSpeed()) / SPATIAL_SCALE);

	 */

	// Windsonic wind
	semiMajorAxis = SPATIAL_SCALE + WIND_SCALE * wind.getSpeed();
	semiMinorAxis = SPATIAL_SCALE / (1 + (WIND_SCALE * wind.getSpeed()) / SPATIAL_SCALE);

	sigma0Upwind[0][0] = semiMajorAxis;
	sigma0Upwind[0][1] = 0;
	sigma0Upwind[1][0] = 0;
	sigma0Upwind[1][1] = semiMinorAxis;

	sigma0Downwind[0][0] = 1 / semiMajorAxis;
	sigma0Downwind[0][1] = 0;
	sigma0Downwind[1][0] = 0;
	sigma0Downwind[1][1] = semiMinorAxis;

	// Simulator angle
	/*
	//alpha is the angle of the wind direction wrt the vector [ 0 1 ]
	double cos_alpha = (wind.getU() * 0 + wind.getV() * 1) / (wind.get2DSpeed() * 1);
	double sin_alpha = sqrt(1 - cos_alpha * cos_alpha);
	 */

	// Windsonic angle
	double cos_alpha = cos(wind.getDirection());
	double sin_alpha = sin(wind.getDirection());



	rotMatrix[0][0] = cos_alpha;
	rotMatrix[0][1] = -sin_alpha;
	rotMatrix[1][0] = sin_alpha;
	rotMatrix[1][1] = cos_alpha;

	sigmaUpwind = (double **)malloc(2 * sizeof(double *));
	sigmaUpwind[0] = (double *)malloc(2 * sizeof(double));
	sigmaUpwind[1] = (double *)malloc(2 * sizeof(double));

	sigmaUpwind[0][0] = semiMajorAxis * cos_alpha * cos_alpha + semiMinorAxis * sin_alpha * sin_alpha;
	sigmaUpwind[0][1] = semiMinorAxis * cos_alpha * sin_alpha - semiMajorAxis * cos_alpha * sin_alpha;
	sigmaUpwind[1][0] = semiMinorAxis * cos_alpha * sin_alpha - semiMajorAxis * cos_alpha * sin_alpha;
	sigmaUpwind[1][1] = semiMinorAxis * cos_alpha * cos_alpha + semiMajorAxis * sin_alpha * sin_alpha;

	invertMatrix(sigmaUpwind);

	sigmaDownwind = (double **)malloc(2 * sizeof(double *));
	sigmaDownwind[0] = (double *)malloc(2 * sizeof(double));
	sigmaDownwind[1] = (double *)malloc(2 * sizeof(double));


	sigmaDownwind[0][0] = semiMajorAxis / 10 * cos_alpha * cos_alpha + semiMinorAxis * sin_alpha * sin_alpha;
	sigmaDownwind[0][1] = semiMinorAxis * cos_alpha * sin_alpha - 1 / semiMajorAxis * cos_alpha * sin_alpha;
	sigmaDownwind[1][0] = semiMinorAxis * cos_alpha * sin_alpha - 1 / semiMajorAxis * cos_alpha * sin_alpha;
	sigmaDownwind[1][1] = semiMinorAxis * cos_alpha * cos_alpha + 1 / semiMajorAxis * sin_alpha * sin_alpha;

	invertMatrix(sigmaDownwind);

}

double KernelFunction::getK(Position pos_x, Position pos_x_prime)
{
	//Position diff = pos_x.diff(pos_x_prime);
	Position diff = pos_x_prime.diff(pos_x);
	double x = diff.getX();
	double y = diff.getY();

	double a, b, c, d;

	if (isUpwind(diff))
	{
		//Upwind
		a = sigmaUpwind[0][0];
		b = sigmaUpwind[0][1];
		c = sigmaUpwind[1][0];
		d = sigmaUpwind[1][1];
	}
	else
	{
		//Downwind
		a = sigmaDownwind[0][0];
		b = sigmaDownwind[0][1];
		c = sigmaDownwind[1][0];
		d = sigmaDownwind[1][1];
	}

	return exp(-1 * sqrt(a * x * x + (b + c) * x * y + d * y * y));

}

bool KernelFunction::isUpwind(Position diff)
{
	// Simulator upwind computation
	/*
	double dot = diff.getX() * wind.getU() + diff.getY() * wind.getV();
	double det = diff.getX() * wind.getV() - diff.getY() * wind.getU();


	double angle = atan2(det, dot);
	return angle > M_PI / 2 || angle < -M_PI / 2;
	*/

	// Windsonic upwind computation
	double diffAngle = atan2(diff.getY(), diff.getX());

	double angle = diffAngle - wind.getDirection();
	return angle < M_PI / 2 && angle > -M_PI / 2;
}

void KernelFunction::invertMatrix(double **matrix)
{
	double a = matrix[0][0];
	double b = matrix[0][1];
	double c = matrix[1][0];
	double d = matrix[1][1];

	double denom = 1 / (a * d - b * c);

	matrix[0][0] = d / denom;
	matrix[0][1] = -1 * b / denom;
	matrix[1][0] = -1 * c / denom;
	matrix[1][1] = a / denom;

}

KernelFunction::~KernelFunction()
{
	if (sigmaDownwind)
	{
		free(sigmaDownwind[0]);
		free(sigmaDownwind[1]);
		free(sigmaDownwind);
	}
	if (sigmaUpwind)
	{
		free(sigmaUpwind[0]);
		free(sigmaUpwind[1]);
		free(sigmaUpwind);
	}

}

