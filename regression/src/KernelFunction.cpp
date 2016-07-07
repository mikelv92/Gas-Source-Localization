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
	sigma0Upwind(0, 0) = semiMajorAxis * 10; //just to stretch a bit more. Should work without it also.
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

}

double KernelFunction::getK(Position pos_x, Position pos_x_prime)
{
	Position diff = pos_x.diff(pos_x_prime);
	double x = diff.getX();
	double y = diff.getY();

	VectorXd diffVec = VectorXd(2);
	diffVec(0) = x;
	diffVec(1) = y;

	double a, b, c, d;

	if (isUpwind(diff))
		return exp(-sqrt((double)(diffVec.transpose() * sigmaUpwind.inverse() * diffVec)(0)));
	else
		return exp(-sqrt((double)(diffVec.transpose() * sigmaDownwind.inverse() * diffVec)(0)));

}

bool KernelFunction::isUpwind(Position diff)
{
	double diffAngle = Utilities::wrapAngle(atan2(diff.getY(), diff.getX()));
	double windAngle = Utilities::wrapAngle(wind.getDirection()); // ?

	double angle = Utilities::wrapAngle(diffAngle - windAngle);
	return !(angle < M_PI / 2 && angle > -M_PI / 2);
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
