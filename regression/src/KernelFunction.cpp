/*
 * KernelFunction.cpp
 *
 *  Created on: May 9, 2016
 *      Author: mikel
 */

#include "regression/KernelFunction.h"

KernelFunction::KernelFunction(Wind w) {
	wind = w;

	// Windsonic wind
	semiMajorAxis = SPATIAL_SCALE + WIND_SCALE * wind.getSpeed();
	semiMinorAxis = SPATIAL_SCALE / (1 + (WIND_SCALE * wind.getSpeed()) / SPATIAL_SCALE);

	// Windsonic angle
	// For some reason maybe we have to add pi/2?
	double windDirection = Utilities::wrapAngle(wind.getDirection());

	double cos_alpha = cos(windDirection);
	double sin_alpha = sin(windDirection);

	MatrixXd rotMatrix = MatrixXd(2, 2);
	rotMatrix(0, 0) = cos_alpha;
	rotMatrix(0, 1) = -sin_alpha;
	rotMatrix(1, 0) = sin_alpha;
	rotMatrix(1, 1) = cos_alpha;

	MatrixXd sigma0Upwind = MatrixXd(2, 2);
	sigma0Upwind(0, 0) = semiMajorAxis * 10;
	sigma0Upwind(0, 1) = 0;
	sigma0Upwind(1, 0) = 0;
	sigma0Upwind(1, 1) = semiMinorAxis;

	MatrixXd sigma0Downwind = MatrixXd(2, 2);
	sigma0Downwind(0, 0) = semiMajorAxis / 10;
	sigma0Downwind(0, 1) = 0;
	sigma0Downwind(1, 0) = 0;
	sigma0Downwind(1, 1) = semiMinorAxis;


	//Is the rotation for the downwind and upwind the same?
	sigmaUpwind = rotMatrix * sigma0Upwind * rotMatrix.inverse();
	sigmaDownwind = rotMatrix * sigma0Downwind * rotMatrix.inverse();

/*
	sigmaUpwind = (double **)malloc(2 * sizeof(double *));
	sigmaUpwind[0] = (double *)malloc(2 * sizeof(double));
	sigmaUpwind[1] = (double *)malloc(2 * sizeof(double));

	sigmaUpwind[0][0] = semiMajorAxis * cos_alpha * cos_alpha + semiMinorAxis * sin_alpha * sin_alpha;
	sigmaUpwind[0][1] = semiMajorAxis * cos_alpha * sin_alpha - semiMinorAxis * cos_alpha * sin_alpha;
	sigmaUpwind[1][0] = semiMajorAxis * cos_alpha * sin_alpha - semiMinorAxis * cos_alpha * sin_alpha;
	sigmaUpwind[1][1] = semiMinorAxis * cos_alpha * cos_alpha + semiMajorAxis * sin_alpha * sin_alpha;

	invertMatrix(sigmaUpwind);

	sigmaDownwind = (double **)malloc(2 * sizeof(double *));
	sigmaDownwind[0] = (double *)malloc(2 * sizeof(double));
	sigmaDownwind[1] = (double *)malloc(2 * sizeof(double));

	double downWindSemiMajorAxis = semiMajorAxis / 10;
	sigmaDownwind[0][0] = downWindSemiMajorAxis * cos_alpha * cos_alpha + semiMinorAxis * sin_alpha * sin_alpha;
	sigmaDownwind[0][1] = downWindSemiMajorAxis * cos_alpha * sin_alpha - semiMinorAxis * cos_alpha * sin_alpha;
	sigmaDownwind[1][0] = downWindSemiMajorAxis * cos_alpha * sin_alpha - semiMinorAxis * cos_alpha * sin_alpha;
	sigmaDownwind[1][1] = semiMinorAxis * cos_alpha * cos_alpha + downWindSemiMajorAxis * sin_alpha * sin_alpha;

	invertMatrix(sigmaDownwind);
*/

}

double KernelFunction::getK(Position pos_x, Position pos_x_prime)
{
	Position diff = pos_x.diff(pos_x_prime);
	//Position diff = pos_x_prime.diff(pos_x);
	double x = diff.getX();
	double y = diff.getY();

	VectorXd diffVec = VectorXd(2);
	diffVec(0) = x;
	diffVec(1) = y;

	double a, b, c, d;

	if (isUpwind(diff))
	{
		//Upwind

		return exp(-sqrt((double)(diffVec.transpose() * sigmaUpwind.inverse() * diffVec)(0)));
/*
		a = sigmaUpwind[0][0];
		b = sigmaUpwind[0][1];
		c = sigmaUpwind[1][0];
		d = sigmaUpwind[1][1];
*/
	}
	else
	{
		//Downwind

		return exp(-sqrt((double)(diffVec.transpose() * sigmaDownwind.inverse() * diffVec)(0)));

/*
		a = sigmaDownwind[0][0];
		b = sigmaDownwind[0][1];
		c = sigmaDownwind[1][0];
		d = sigmaDownwind[1][1];
*/
	}

//	return exp(-1 * sqrt(a * x * x + (b + c) * x * y + d * y * y));

}

bool KernelFunction::isUpwind(Position diff)
{
	double diffAngle = atan2(diff.getY(), diff.getX());
	double windAngle = Utilities::wrapAngle(wind.getDirection()); // ?

	double angle = diffAngle - windAngle;
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
/*
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
*/

}

