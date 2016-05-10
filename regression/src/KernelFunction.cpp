/*
 * KernelFunction.cpp
 *
 *  Created on: May 9, 2016
 *      Author: mikel
 */

#include "regression/KernelFunction.h"


KernelFunction::KernelFunction(Wind w) {
	wind = w;

	semiMajorAxis = SPATIAL_SCALE + WIND_SCALE * wind.get2DSpeed();
	semiMinorAxis = SPATIAL_SCALE / (1 + (WIND_SCALE * wind.get2DSpeed()) / SPATIAL_SCALE);

	sigma0Upwind[0][0] = semiMajorAxis;
	sigma0Upwind[0][1] = 0;
	sigma0Upwind[1][0] = 0;
	sigma0Upwind[1][1] = semiMinorAxis;

	sigma0Downwind[0][0] = semiMajorAxis / REDUCE_FACTOR;
	sigma0Downwind[0][1] = 0;
	sigma0Downwind[1][0] = 0;
	sigma0Downwind[1][1] = semiMinorAxis;

	//alpha is the angle of the wind direction wrt the vector [ 0 1 ]
	float cos_alpha = (wind.getU() * 0 + wind.getV() * 1) / (wind.get2DSpeed() * 1);
	float sin_alpha = sqrt(1 - cos_alpha * cos_alpha);

	rotMatrix[0][0] = cos_alpha;
	rotMatrix[0][1] = -1 * sin_alpha;
	rotMatrix[1][0] = sin_alpha;
	rotMatrix[1][1] = cos_alpha;

	sigmaUpwind[0][0] = semiMajorAxis * cos_alpha * cos_alpha + semiMinorAxis * sin_alpha * sin_alpha;
	sigmaUpwind[0][1] = semiMajorAxis * cos_alpha * sin_alpha - semiMinorAxis * cos_alpha * sin_alpha;
	sigmaUpwind[1][0] = semiMajorAxis * cos_alpha * sin_alpha - semiMinorAxis * cos_alpha * sin_alpha;
	sigmaUpwind[1][1] = semiMinorAxis * cos_alpha * cos_alpha + semiMajorAxis * sin_alpha * sin_alpha;

	sigmaDownwind[0][0] = semiMajorAxis / REDUCE_FACTOR * cos_alpha * cos_alpha + semiMinorAxis * sin_alpha * sin_alpha;
	sigmaDownwind[0][1] = semiMajorAxis / REDUCE_FACTOR * cos_alpha * sin_alpha - semiMinorAxis * cos_alpha * sin_alpha;
	sigmaDownwind[1][0] = semiMajorAxis / REDUCE_FACTOR * cos_alpha * sin_alpha - semiMinorAxis * cos_alpha * sin_alpha;
	sigmaDownwind[1][1] = semiMinorAxis / REDUCE_FACTOR * cos_alpha * cos_alpha + semiMajorAxis * sin_alpha * sin_alpha;

}

float KernelFunction::getK(Position pos_x, Position pos_x_prime)
{
	float k = 0;
	Position diff = pos_x.diff(pos_x_prime);
	float x = diff.getX();
	float y = diff.getY();

	//TODO Check angle of diff and the wind

	if (isUpwind(diff))
	{
		//Upwind
		float** invertedUpwindSigma;
		invertedUpwindSigma = invertMatrix(sigmaUpwind);
		float a = invertedUpwindSigma[0][0];
		float b = invertedUpwindSigma[0][1];
		float c = invertedUpwindSigma[1][0];
		float d = invertedUpwindSigma[1][1];


		k = exp(-1 * sqrt(a * x * x + (b + c) * x * y + d * y * y));
	}
	else
	{
		//Downwind
		float** invertedDownwindSigma;
		invertedDownwindSigma = invertMatrix(sigmaDownwind);
		float a = invertedDownwindSigma[0][0];
		float b = invertedDownwindSigma[0][1];
		float c = invertedDownwindSigma[1][0];
		float d = invertedDownwindSigma[1][1];

		k = exp(-1 * sqrt(a * x * x + (b + c) * x * y + d * y * y));
	}
	return k;

}

bool KernelFunction::isUpwind(Position diff)
{
	double dot = diff.getX() * wind.getU() + diff.getY() * wind.getV();
	double det = diff.getX() * wind.getV() - diff.getY() * wind.getU();

	double angle = atan2(det, dot);

	return angle > M_PI / 2 || angle < -1 * M_PI / 2;
}

float** KernelFunction::invertMatrix(float matrix[2][2])
{
	float a = matrix[0][0];
	float b = matrix[0][1];
	float c = matrix[1][0];
	float d = matrix[1][1];

	float denom = 1 / (a * d - b * c);
	float** inverted_matrix;

	inverted_matrix[0][0] = d / denom;
	inverted_matrix[0][1] = -1 * b / denom;
	inverted_matrix[1][0] = -1 * c / denom;
	inverted_matrix[1][1] = a / denom;
	return inverted_matrix;
}

KernelFunction::~KernelFunction() {
}

